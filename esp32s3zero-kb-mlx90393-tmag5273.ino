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

// --- MLX90393 SLIDER (Right Side: C/D) ---
Adafruit_MLX90393 mlx = Adafruit_MLX90393();
bool  mlxReady  = false;
float mlxBaselineX = 0, mlxBaselineY = 0, mlxBaselineZ = 0;

const float MLX_UP_THRESHOLD   = 450.0f;  
const float MLX_DOWN_THRESHOLD = -450.0f; 
const float MLX_DEADZONE       = 150.0f;  

int           mlxSliderState   = 0; 
float         lastMlxDebugDx   = 0; 
unsigned long lastMlxTimeMs    = 0;

#define MLX_FAIL_LIMIT    5
#define MLX_RECOVERY_MS   3000
int           mlxFailCount  = 0;
unsigned long mlxLastFailMs = 0;

// --- TMAG5273 SLIDER (Left Side: A/B) ---
TMAG5273 tmag;
bool  tmagReady = false;
float tmagBaselineX = 0; // Assuming the slider moves along the sensor's X-axis

// TMAG values are in mT, so the scale is much smaller than the MLX. 
// Tweak these based on your Telnet debug outputs!
const float TMAG_UP_THRESHOLD   = 2.0f;  
const float TMAG_DOWN_THRESHOLD = -2.0f; 
const float TMAG_DEADZONE       = 0.5f;  

int           tmagSliderState = 0;
float         lastTmagDebugDx = 0;
unsigned long lastTmagTimeMs  = 0;
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
    
    float bx = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      bx += tmag.getXData();
      delay(10);
    }
    tmagBaselineX = bx / BASELINE_SAMPLES;
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
  // 2. MLX Slider Logic (Right Slider - C/D)
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
            mlxSliderState  = 0;
            debugPrintln("[MLX] Soft recovery OK.");
          }
        }
      }
    } else {
      mlxFailCount = 0;
      float dx = x - mlxBaselineX; 

      if (abs(dx - lastMlxDebugDx) > 30.0f) {
          // debugPrintf("[CALIB] MLX DX: %.1f\n", dx);
          lastMlxDebugDx = dx;
      }

      if (millis() - lastMlxTimeMs >= SLIDER_COOLDOWN_MS) {
        if (mlxSliderState == 0) {
          if (dx > MLX_UP_THRESHOLD) {
            mlxSliderState = 1;
            lastMlxTimeMs = millis();
            Keyboard.write('c');  
            debugPrintln("MLX UP -> 'c'");
          } else if (dx < MLX_DOWN_THRESHOLD) {
            mlxSliderState = -1;
            lastMlxTimeMs = millis();
            Keyboard.write('d');  
            debugPrintln("MLX DOWN -> 'd'");
          }
        } else if (mlxSliderState == 1 && dx < MLX_DEADZONE) {
          mlxSliderState = 0; 
          lastMlxTimeMs = millis();
          debugPrintln("MLX CENTER from UP");
        } else if (mlxSliderState == -1 && dx > -MLX_DEADZONE) {
          mlxSliderState = 0; 
          lastMlxTimeMs = millis();
          debugPrintln("MLX CENTER from DOWN");
        }
      }
    }
  }

  // ===========================================================
  // 3. TMAG Slider Logic (Left Slider - A/B)
  // ===========================================================
  if (tmagReady && (millis() - lastTmagReadMs >= TMAG_INTERVAL_MS)) {
    lastTmagReadMs = millis();
    
    // Read the primary axis of movement
    // Note: If you mount the TMAG rotated 90 degrees compared to the sketch, 
    // you might need to change getXData() to getYData() here!
    float currentX = tmag.getXData();
    float tdx = currentX - tmagBaselineX;

    // Telnet Debugging - Crucial for finding your thresholds
    if (abs(tdx - lastTmagDebugDx) > 0.5f) {
      debugPrintf("[TMAG CALIB] Slider DX: %.2f\n", tdx);
      lastTmagDebugDx = tdx;
    }

    if (millis() - lastTmagTimeMs >= SLIDER_COOLDOWN_MS) {
      if (tmagSliderState == 0) {
        if (tdx > TMAG_UP_THRESHOLD) {
          tmagSliderState = 1;
          lastTmagTimeMs = millis();
          Keyboard.write('a');  
          debugPrintln("TMAG UP -> 'a'");
        } else if (tdx < TMAG_DOWN_THRESHOLD) {
          tmagSliderState = -1;
          lastTmagTimeMs = millis();
          Keyboard.write('b');  
          debugPrintln("TMAG DOWN -> 'b'");
        }
      } else if (tmagSliderState == 1 && tdx < TMAG_DEADZONE) {
        tmagSliderState = 0; 
        lastTmagTimeMs = millis();
        debugPrintln("TMAG CENTER from UP");
      } else if (tmagSliderState == -1 && tdx > -TMAG_DEADZONE) {
        tmagSliderState = 0; 
        lastTmagTimeMs = millis();
        debugPrintln("TMAG CENTER from DOWN");
      }
    }
  }
}
