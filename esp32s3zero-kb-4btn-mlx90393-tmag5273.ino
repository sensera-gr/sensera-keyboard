#include "APWifiConfig.h"
#include "USB.h"
#include "USBHIDKeyboard.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <SparkFun_TMAG5273_Arduino_Library.h> 

// --- BLE & WIFI SETTINGS ---
#define SERVICE_UUID        "4fa2c730-1a8a-4600-a54b-d72b22e11895"
#define COMMAND_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define KEYBOARD_CHAR_UUID  "cba1d466-344c-4be3-ab3f-189f80dd7518"

USBHIDKeyboard Keyboard;
WiFiServer     telnetServer(23);
WiFiClient     telnetClient;

// --- HARDWARE PINS ---
#define I2C_SDA 8
#define I2C_SCL 9

#define BASELINE_SAMPLES   20
#define SLIDER_COOLDOWN_MS 150 

// =============================================================
//  MLX90393 CROSS SLIDER (Right Side)
// =============================================================
Adafruit_MLX90393 mlx = Adafruit_MLX90393();
bool  mlxReady  = false;
float mlxBaselineX = 0, mlxBaselineY = 0, mlxBaselineZ = 0;

// Thresholds for X (Up/Down) and Y (Left/Right)
const float MLX_X_POS_THRES = 450.0f;  // UP
const float MLX_X_NEG_THRES = -450.0f; // DOWN
const float MLX_Y_POS_THRES = 450.0f;  // RIGHT
const float MLX_Y_NEG_THRES = -450.0f; // LEFT
const float MLX_DEADZONE    = 150.0f;  

int           mlxStateX      = 0; 
int           mlxStateY      = 0; 
float         lastMlxDebugX  = 0; 
float         lastMlxDebugY  = 0; 
unsigned long lastMlxTimeX   = 0;
unsigned long lastMlxTimeY   = 0;

#define MLX_FAIL_LIMIT    5
#define MLX_RECOVERY_MS   3000
int           mlxFailCount  = 0;
unsigned long mlxLastFailMs = 0;

// =============================================================
//  TMAG5273 CROSS SLIDER (Left Side)
// =============================================================
TMAG5273 tmag;
bool  tmagReady = false;
float tmagBaselineX = 0; 
float tmagBaselineY = 0; 

// Thresholds for X (Up/Down) and Y (Left/Right)
const float TMAG_X_POS_THRES = 2.0f;  // UP
const float TMAG_X_NEG_THRES = -2.0f; // DOWN
const float TMAG_Y_POS_THRES = 2.0f;  // RIGHT
const float TMAG_Y_NEG_THRES = -2.0f; // LEFT
const float TMAG_DEADZONE    = 0.5f;  

int           tmagStateX      = 0;
int           tmagStateY      = 0;
float         lastTmagDebugX  = 0;
float         lastTmagDebugY  = 0;
unsigned long lastTmagTimeX   = 0;
unsigned long lastTmagTimeY   = 0;

#define TMAG_INTERVAL_MS 20
unsigned long lastTmagReadMs  = 0;


// --- OTA / TELNET ---
bool otaEnabled    = false;
bool lastOtaState  = false;
bool telnetEnabled = false;

// =============================================================
//  TELNET HELPERS
// =============================================================
void debugPrint(String msg) {
  if (telnetEnabled && telnetClient && telnetClient.connected())
    telnetClient.print(msg);
}
void debugPrintln(String msg) {
  if (telnetEnabled && telnetClient && telnetClient.connected())
    telnetClient.println(msg);
}
void debugPrintf(const char *fmt, ...) {
  if (telnetEnabled && telnetClient && telnetClient.connected()) {
    va_list arg; va_start(arg, fmt);
    char tmp[128]; vsnprintf(tmp, sizeof(tmp), fmt, arg);
    telnetClient.print(tmp); va_end(arg);
  }
}

// =============================================================
//  BLE CALLBACKS
// =============================================================
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pC) {
    String cmd = String(pC->getValue().c_str());
    cmd.trim();
    if (cmd == "1" || cmd == "OTA_ON") {
      otaEnabled = true; telnetEnabled = true;
    } else if (cmd == "0" || cmd == "OTA_OFF") {
      otaEnabled = false; telnetEnabled = false;
      if (telnetClient) telnetClient.stop();
    }
  }
};

class KeyboardCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pC) {
    String value = String(pC->getValue().c_str());
    if (value.length() > 0) {
      Keyboard.print(value);
      debugPrint("Typed: "); debugPrintln(value);
    }
  }
};

// =============================================================
//  WIFI AP
// =============================================================
void startWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  ArduinoOTA.begin();
  if (telnetEnabled) telnetServer.begin();
}

void stopWiFiAP() {
  if (telnetClient) telnetClient.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

// =============================================================
//  MLX SLIDER RECOVERY
// =============================================================
bool mlxSoftRecover() {
  mlx.setGain(MLX90393_GAIN_1X);
  mlx.setResolution(MLX90393_X, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Y, MLX90393_RES_17);
  mlx.setResolution(MLX90393_Z, MLX90393_RES_16);
  mlx.setOversampling(MLX90393_OSR_0);
  mlx.setFilter(MLX90393_FILTER_0);

  float x, y, z;
  float bx = 0, by = 0, bz = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    if (!mlx.readData(&x, &y, &z)) return false;
    bx += x; by += y; bz += z;
    delay(10);
  }
  mlxBaselineX = bx / BASELINE_SAMPLES;
  mlxBaselineY = by / BASELINE_SAMPLES;
  mlxBaselineZ = bz / BASELINE_SAMPLES;
  return true;
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  delay(800);
  WiFi.mode(WIFI_OFF);

  BLEDevice::init(ble_name); 
  
  BLEServer  *pServer  = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCmdChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCmdChar->setCallbacks(new CommandCallbacks());
  pCmdChar->setValue("0");

  BLECharacteristic *pKbdChar = pService->createCharacteristic(
    KEYBOARD_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pKbdChar->setCallbacks(new KeyboardCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // MLX Init
  if (mlx.begin_I2C(0x0C, &Wire)) {
    mlxReady = true;
    mlxSoftRecover(); 
  }

  // --- Init TMAG (0x35) ---
  if (tmag.begin(0x35, Wire)) {
    tmagReady = true;
    tmag.setConvAvg(TMAG5273_X32_CONVERSION); 
    
    float bx = 0, by = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      bx += tmag.getXData();
      by += tmag.getYData(); // Capture Y baseline for L/R
      delay(10);
    }
    tmagBaselineX = bx / BASELINE_SAMPLES;
    tmagBaselineY = by / BASELINE_SAMPLES;
  }

  Keyboard.begin();
  USB.begin();
}

// =============================================================
//  LOOP
// =============================================================
void loop() {

  // 1. OTA / Telnet
  if (otaEnabled != lastOtaState) {
    if (otaEnabled) startWiFiAP(); else stopWiFiAP();
    lastOtaState = otaEnabled;
  }
  if (otaEnabled) {
    ArduinoOTA.handle();
    if (telnetEnabled && telnetServer.hasClient()) {
      if (!telnetClient || !telnetClient.connected()) {
        if (telnetClient) telnetClient.stop();
        telnetClient = telnetServer.available();
        telnetClient.println("--- Debugger Connected ---");
      } else {
        telnetServer.available().stop();
      }
    }
  }

  // ===========================================================
  // 2. MLX CROSS LOGIC (Right Side)
  // ===========================================================
  if (mlxReady) {
    float x, y, z;
    if (!mlx.readData(&x, &y, &z)) {
      mlxFailCount++;
      if (mlxFailCount >= MLX_FAIL_LIMIT) {
        if (millis() - mlxLastFailMs >= MLX_RECOVERY_MS) {
          mlxLastFailMs = millis();
          debugPrintln("[MLX] Soft recovery attempt...");
          if (mlxSoftRecover()) {
            mlxFailCount = 0;
            mlxStateX = 0;
            mlxStateY = 0;
            debugPrintln("[MLX] Soft recovery OK.");
          }
        }
      }
    } else {
      mlxFailCount = 0;
      float dx = x - mlxBaselineX; 
      float dy = y - mlxBaselineY; 

      if (abs(dx - lastMlxDebugX) > 30.0f) {
          // debugPrintf("[CALIB] MLX DX: %.1f\n", dx);
          lastMlxDebugX = dx;
      }
      if (abs(dy - lastMlxDebugY) > 30.0f) {
          // debugPrintf("[CALIB] MLX DY: %.1f\n", dy);
          lastMlxDebugY = dy;
      }

      // --- MLX X-Axis (Up/Down) ---
      if (millis() - lastMlxTimeX >= SLIDER_COOLDOWN_MS) {
        if (mlxStateX == 0) {
          if (dx > MLX_X_POS_THRES) {
            mlxStateX = 1; lastMlxTimeX = millis();
            Keyboard.write('c'); debugPrintln("MLX UP -> 'c'");
          } else if (dx < MLX_X_NEG_THRES) {
            mlxStateX = -1; lastMlxTimeX = millis();
            Keyboard.write('d'); debugPrintln("MLX DOWN -> 'd'");
          }
        } else if (mlxStateX == 1 && dx < MLX_DEADZONE) {
          mlxStateX = 0; lastMlxTimeX = millis(); debugPrintln("MLX Reset X-Axis");
        } else if (mlxStateX == -1 && dx > -MLX_DEADZONE) {
          mlxStateX = 0; lastMlxTimeX = millis(); debugPrintln("MLX Reset X-Axis");
        }
      }

      // --- MLX Y-Axis (Left/Right) ---
      if (millis() - lastMlxTimeY >= SLIDER_COOLDOWN_MS) {
        if (mlxStateY == 0) {
          if (dy > MLX_Y_POS_THRES) {
            mlxStateY = 1; lastMlxTimeY = millis();
            Keyboard.write('f'); debugPrintln("MLX RIGHT -> 'f'");
          } else if (dy < MLX_Y_NEG_THRES) {
            mlxStateY = -1; lastMlxTimeY = millis();
            Keyboard.write('e'); debugPrintln("MLX LEFT -> 'e'");
          }
        } else if (mlxStateY == 1 && dy < MLX_DEADZONE) {
          mlxStateY = 0; lastMlxTimeY = millis(); debugPrintln("MLX Reset Y-Axis");
        } else if (mlxStateY == -1 && dy > -MLX_DEADZONE) {
          mlxStateY = 0; lastMlxTimeY = millis(); debugPrintln("MLX Reset Y-Axis");
        }
      }
    }
  }

  // ===========================================================
  // 3. TMAG CROSS LOGIC (Left Side)
  // ===========================================================
  if (tmagReady && (millis() - lastTmagReadMs >= TMAG_INTERVAL_MS)) {
    lastTmagReadMs = millis();
    
    float currentX = tmag.getXData();
    float currentY = tmag.getYData();
    float tdx = currentX - tmagBaselineX;
    float tdy = currentY - tmagBaselineY;

    if (abs(tdx - lastTmagDebugX) > 0.5f) {
      // debugPrintf("[TMAG] DX: %.2f\n", tdx);
      lastTmagDebugX = tdx;
    }
    if (abs(tdy - lastTmagDebugY) > 0.5f) {
      // debugPrintf("[TMAG] DY: %.2f\n", tdy);
      lastTmagDebugY = tdy;
    }

    // --- TMAG X-Axis (Up/Down) ---
    if (millis() - lastTmagTimeX >= SLIDER_COOLDOWN_MS) {
      if (tmagStateX == 0) {
        if (tdx > TMAG_X_POS_THRES) {
          tmagStateX = 1; lastTmagTimeX = millis();
          Keyboard.write('a'); debugPrintln("TMAG UP -> 'a'");
        } else if (tdx < TMAG_X_NEG_THRES) {
          tmagStateX = -1; lastTmagTimeX = millis();
          Keyboard.write('b'); debugPrintln("TMAG DOWN -> 'b'");
        }
      } else if (tmagStateX == 1 && tdx < TMAG_DEADZONE) {
        tmagStateX = 0; lastTmagTimeX = millis(); debugPrintln("TMAG Reset X-Axis");
      } else if (tmagStateX == -1 && tdx > -TMAG_DEADZONE) {
        tmagStateX = 0; lastTmagTimeX = millis(); debugPrintln("TMAG Reset X-Axis");
      }
    }

    // --- TMAG Y-Axis (Left/Right) ---
    if (millis() - lastTmagTimeY >= SLIDER_COOLDOWN_MS) {
      if (tmagStateY == 0) {
        if (tdy > TMAG_Y_POS_THRES) {
          tmagStateY = 1; lastTmagTimeY = millis();
          Keyboard.write('r'); debugPrintln("TMAG RIGHT -> 'r'");
        } else if (tdy < TMAG_Y_NEG_THRES) {
          tmagStateY = -1; lastTmagTimeY = millis();
          Keyboard.write('l'); debugPrintln("TMAG LEFT -> 'l'");
        }
      } else if (tmagStateY == 1 && tdy < TMAG_DEADZONE) {
        tmagStateY = 0; lastTmagTimeY = millis(); debugPrintln("TMAG Reset Y-Axis");
      } else if (tmagStateY == -1 && tdy > -TMAG_DEADZONE) {
        tmagStateY = 0; lastTmagTimeY = millis(); debugPrintln("TMAG Reset Y-Axis");
      }
    }
  }
}
