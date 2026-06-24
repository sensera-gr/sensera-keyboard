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

// --- MLX90393 SLIDER ---
Adafruit_MLX90393 mlx = Adafruit_MLX90393();
#define BASELINE_SAMPLES   20
float baselineX = 0, baselineY = 0, baselineZ = 0;
bool  mlxReady  = false;

const float SLIDER_UP_THRESHOLD   = 450.0f;  
const float SLIDER_DOWN_THRESHOLD = -450.0f; 
const float SLIDER_DEADZONE       = 150.0f;  

int           sliderState      = 0; 
float         lastDebugDx      = 0; 
unsigned long lastSliderTimeMs = 0;
#define SLIDER_COOLDOWN_MS 150 

#define MLX_FAIL_LIMIT    5
#define MLX_RECOVERY_MS   3000
int           mlxFailCount  = 0;
unsigned long mlxLastFailMs = 0;

// --- HALL SENSORS ---
const int   HALL_PINS[]  = {1, 2};
const int   HALL_COUNT   = 2;
const char  HALL_KEYS[]  = {'a', 'b'};
const char* HALL_NAMES[] = {"HALL-A", "HALL-B"};

#define ADC_RESOLUTION   12
#define ADC_ATTENUATION  ADC_11db
#define HYPER_SAMPLES    128

// EVEN LOWER THRESHOLDS
const float PRESS_THRESHOLD   = 6.0f; 
const float RELEASE_THRESHOLD = 4.0f;  
const int   DEBOUNCE_TARGET   = 2;

int   triggerCount[HALL_COUNT] = {0, 0};
float hallBaseline[HALL_COUNT];
bool  isPressed[HALL_COUNT]    = {false, false};
bool  wasPressed[HALL_COUNT]   = {false, false};
float lastHallDebug[HALL_COUNT] = {0, 0}; // Used to prevent Telnet spam

unsigned long lastHallMs = 0;
#define HALL_INTERVAL_MS 20

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
//  ADC
// =============================================================
float readADC_HighRes(int pin) {
  long sum = 0;
  for (int i = 0; i < HYPER_SAMPLES; i++) sum += analogRead(pin);
  return (float)sum / (float)HYPER_SAMPLES;
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
//  MLX SLIDER HELPERS
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
  baselineX = bx / BASELINE_SAMPLES;
  baselineY = by / BASELINE_SAMPLES;
  baselineZ = bz / BASELINE_SAMPLES;
  return true;
}

void processSlider() {
  if (!mlxReady) return;

  float x, y, z;
  if (!mlx.readData(&x, &y, &z)) {
    mlxFailCount++;
    if (mlxFailCount >= MLX_FAIL_LIMIT) {
      if (millis() - mlxLastFailMs >= MLX_RECOVERY_MS) {
        mlxLastFailMs = millis();
        debugPrintln("[MLX] Soft recovery attempt...");
        if (mlxSoftRecover()) {
          mlxFailCount = 0;
          sliderState  = 0;
          debugPrintln("[MLX] Soft recovery OK.");
        }
      }
    }
    return;
  }

  mlxFailCount = 0;
  float dx = x - baselineX; 

  if (abs(dx - lastDebugDx) > 30.0f) {
      // Temporarily disabled to let you focus on Hall logs, uncomment if needed
      // debugPrintf("[CALIBRATION] Slider raw DX: %.1f\n", dx);
      lastDebugDx = dx;
  }

  if (millis() - lastSliderTimeMs < SLIDER_COOLDOWN_MS) return; 

  if (sliderState == 0) {
    if (dx > SLIDER_UP_THRESHOLD) {
      sliderState = 1;
      lastSliderTimeMs = millis();
      Keyboard.write('c');  
      debugPrintln("Slider UP -> 'c'");
    } else if (dx < SLIDER_DOWN_THRESHOLD) {
      sliderState = -1;
      lastSliderTimeMs = millis();
      Keyboard.write('d');  
      debugPrintln("Slider DOWN -> 'd'");
    }
  } else if (sliderState == 1) {
    if (dx < SLIDER_DEADZONE) {
      sliderState = 0; 
      lastSliderTimeMs = millis();
      debugPrintln("Slider returned to CENTER from UP");
    }
  } else if (sliderState == -1) {
    if (dx > -SLIDER_DEADZONE) {
      sliderState = 0; 
      lastSliderTimeMs = millis();
      debugPrintln("Slider returned to CENTER from DOWN");
    }
  }
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
    mlx.setGain(MLX90393_GAIN_1X);
    mlx.setResolution(MLX90393_X, MLX90393_RES_17);
    mlx.setResolution(MLX90393_Y, MLX90393_RES_17);
    mlx.setResolution(MLX90393_Z, MLX90393_RES_16);
    mlx.setOversampling(MLX90393_OSR_0);
    mlx.setFilter(MLX90393_FILTER_0);

    float x, y, z;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      mlx.readData(&x, &y, &z);
      baselineX += x; baselineY += y; baselineZ += z;
      delay(10);
    }
    baselineX /= BASELINE_SAMPLES;
    baselineY /= BASELINE_SAMPLES;
    baselineZ /= BASELINE_SAMPLES;
  } else {
    mlxReady = false;
  }

  // Hall sensors
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);
  for (int i = 0; i < HALL_COUNT; i++) pinMode(HALL_PINS[i], INPUT);
  for (int i = 0; i < HALL_COUNT; i++) hallBaseline[i] = readADC_HighRes(HALL_PINS[i]);

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

  // 2. MLX Slider
  processSlider();

  // 3. Hall Sensors
  if (millis() - lastHallMs >= HALL_INTERVAL_MS) {
    lastHallMs = millis();

    float currentRaw[HALL_COUNT];
    float deviation[HALL_COUNT];
    float globalAverageDeviation = 0;

    for (int i = 0; i < HALL_COUNT; i++) {
      currentRaw[i] = readADC_HighRes(HALL_PINS[i]);
      deviation[i]  = currentRaw[i] - hallBaseline[i];
      globalAverageDeviation += deviation[i];
    }
    globalAverageDeviation /= HALL_COUNT;

    for (int i = 0; i < HALL_COUNT; i++) {
      float isolatedSignal = deviation[i] - globalAverageDeviation;

      // Print to Telnet if the magnet is causing a noticeable change
      if (abs(isolatedSignal - lastHallDebug[i]) > 3.0f) {
        debugPrintf("[HALL CALIB] %s Signal: %.1f\n", HALL_NAMES[i], isolatedSignal);
        lastHallDebug[i] = isolatedSignal;
      }

      if (!isPressed[i]) {
        if (isolatedSignal < -PRESS_THRESHOLD) {
          triggerCount[i]++;
          if (triggerCount[i] >= DEBOUNCE_TARGET) {
            isPressed[i]    = true;
            triggerCount[i] = 0;
          }
        } else {
          triggerCount[i] = 0;
          // FREEZE BASELINE: Only auto-calibrate if the signal is virtually zero (no magnet nearby)
          if (isolatedSignal > -2.0f && isolatedSignal < 2.0f) {
            hallBaseline[i] = (hallBaseline[i] * 0.9995f) + (currentRaw[i] * 0.0005f);
          }
        }
      } else {
        if (isolatedSignal > -RELEASE_THRESHOLD) {
          triggerCount[i]++;
          if (triggerCount[i] >= DEBOUNCE_TARGET) {
            isPressed[i]    = false;
            triggerCount[i] = 0;
          }
        } else {
          triggerCount[i] = 0;
        }
      }

      if (isPressed[i] && !wasPressed[i]) {
        Keyboard.write(HALL_KEYS[i]);
        debugPrintln(String("Pressed ") + HALL_NAMES[i]
                     + " -> '" + HALL_KEYS[i] + "'");
      }
      wasPressed[i] = isPressed[i];
    }
  }
}
