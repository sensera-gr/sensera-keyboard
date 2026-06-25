#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90393.h>

// --- HARDWARE PINS ---
#define I2C_SDA 8
#define I2C_SCL 9

// =============================================================
//  MLX90393 SLIDER (Right Side: C/D)
// =============================================================
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

// =============================================================
//  ANALOG HALL SENSORS (Left Side: A/B)
// =============================================================
const int   HALL_PINS[]  = {1, 2};
const int   HALL_COUNT   = 2;
const char  HALL_KEYS[]  = {'a', 'b'};
const char* HALL_NAMES[] = {"HALL-A", "HALL-B"};

#define ADC_RESOLUTION   12
#define ADC_ATTENUATION  ADC_11db
#define HYPER_SAMPLES    128

const float PRESS_THRESHOLD   = 6.0f; 
const float RELEASE_THRESHOLD = 4.0f;  
const int   DEBOUNCE_TARGET   = 2;

int   triggerCount[HALL_COUNT] = {0, 0};
float hallBaseline[HALL_COUNT];
bool  isPressed[HALL_COUNT]    = {false, false};
bool  wasPressed[HALL_COUNT]   = {false, false};
float lastHallDebug[HALL_COUNT] = {0, 0}; 

unsigned long lastHallMs = 0;
#define HALL_INTERVAL_MS 20

// =============================================================
//  ADC HELPER
// =============================================================
float readADC_HighRes(int pin) {
  long sum = 0;
  for (int i = 0; i < HYPER_SAMPLES; i++) sum += analogRead(pin);
  return (float)sum / (float)HYPER_SAMPLES;
}

// =============================================================
//  MLX RECOVERY HELPER
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

// =============================================================
//  MLX PROCESS LOGIC
// =============================================================
void processSlider() {
  if (!mlxReady) return;

  float x, y, z;
  if (!mlx.readData(&x, &y, &z)) {
    mlxFailCount++;
    if (mlxFailCount >= MLX_FAIL_LIMIT) {
      if (millis() - mlxLastFailMs >= MLX_RECOVERY_MS) {
        mlxLastFailMs = millis();
        Serial.println("[MLX] Soft recovery attempt...");
        if (mlxSoftRecover()) {
          mlxFailCount = 0;
          sliderState  = 0;
          Serial.println("[MLX] Soft recovery OK.");
        }
      }
    }
    return;
  }

  mlxFailCount = 0;
  float dx = x - baselineX; 

  // Print raw values when they change significantly
  if (abs(dx - lastDebugDx) > 30.0f) {
      Serial.printf("[MLX CALIB] Slider raw DX: %.1f\n", dx);
      lastDebugDx = dx;
  }

  if (millis() - lastSliderTimeMs < SLIDER_COOLDOWN_MS) return; 

  if (sliderState == 0) {
    if (dx > SLIDER_UP_THRESHOLD) {
      sliderState = 1;
      lastSliderTimeMs = millis();
      Serial.println(">> [TRIGGER] MLX Slider UP   -> 'c'");
    } else if (dx < SLIDER_DOWN_THRESHOLD) {
      sliderState = -1;
      lastSliderTimeMs = millis();
      Serial.println(">> [TRIGGER] MLX Slider DOWN -> 'd'");
    }
  } else if (sliderState == 1) {
    if (dx < SLIDER_DEADZONE) {
      sliderState = 0; 
      lastSliderTimeMs = millis();
      Serial.println("<< [RESET] MLX Slider returned to CENTER from UP");
    }
  } else if (sliderState == -1) {
    if (dx > -SLIDER_DEADZONE) {
      sliderState = 0; 
      lastSliderTimeMs = millis();
      Serial.println("<< [RESET] MLX Slider returned to CENTER from DOWN");
    }
  }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(2000); // Give time for Serial Monitor to connect
  Serial.println("\n--- Original Sensor Calibration Mode ---");

  // I2C & MLX Init
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  if (mlx.begin_I2C(0x0C, &Wire)) {
    mlxReady = true;
    if (mlxSoftRecover()) {
      Serial.println("[OK] MLX90393 Initialized and Baselined.");
    } else {
      Serial.println("[WARN] MLX90393 Baseline failed.");
    }
  } else {
    Serial.println("[FAIL] MLX90393 Not Found!");
  }

  // Analog Hall Init
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);
  for (int i = 0; i < HALL_COUNT; i++) {
    pinMode(HALL_PINS[i], INPUT);
    hallBaseline[i] = readADC_HighRes(HALL_PINS[i]);
  }
  Serial.println("[OK] Analog Hall Sensors Initialized and Baselined.");
  Serial.println("----------------------------------------\n");
}

// =============================================================
//  LOOP
// =============================================================
void loop() {

  // 1. MLX Slider
  processSlider();

  // 2. Hall Sensors
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

      // Print raw signal to Serial if the magnet is causing a noticeable change
      if (abs(isolatedSignal - lastHallDebug[i]) > 3.0f) {
        Serial.printf("[HALL CALIB] %s Signal: %.1f\n", HALL_NAMES[i], isolatedSignal);
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
        Serial.printf(">> [TRIGGER] Pressed %s -> '%c'\n", HALL_NAMES[i], HALL_KEYS[i]);
      } else if (!isPressed[i] && wasPressed[i]) {
        Serial.printf("<< [RESET] Released %s\n", HALL_NAMES[i]);
      }
      
      wasPressed[i] = isPressed[i];
    }
  }
}
