#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <SparkFun_TMAG5273_Arduino_Library.h> 

// --- HARDWARE PINS ---
#define I2C_SDA 8
#define I2C_SCL 9

#define BASELINE_SAMPLES   20
#define SLIDER_COOLDOWN_MS 150 

// =============================================================
//  MLX90393 SLIDER (Right Side: C/D)
// =============================================================
Adafruit_MLX90393 mlx = Adafruit_MLX90393();
bool  mlxReady  = false;
float mlxBaselineX = 0, mlxBaselineY = 0, mlxBaselineZ = 0;

// CALIBRATION THRESHOLDS (Raw Bits)
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

// =============================================================
//  TMAG5273 SLIDER (Left Side: A/B)
// =============================================================
TMAG5273 tmag;
bool  tmagReady = false;
float tmagBaselineX = 0; 

// CALIBRATION THRESHOLDS (Milliteslas - mT)
const float TMAG_UP_THRESHOLD   = 2.0f;  
const float TMAG_DOWN_THRESHOLD = -2.0f; 
const float TMAG_DEADZONE       = 0.5f;  

int           tmagSliderState = 0;
float         lastTmagDebugDx = 0;
unsigned long lastTmagTimeMs  = 0;
#define TMAG_INTERVAL_MS 20
unsigned long lastTmagReadMs  = 0;

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
  Serial.begin(115200);
  delay(2000); // Give time for Serial Monitor to connect
  Serial.println("\n--- Keyboard Sensor Calibration Mode ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // --- Init MLX (0x0C) ---
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

  // --- Init TMAG (0x35) ---
  if (tmag.begin(0x35, Wire)) {
    tmagReady = true;
    tmag.setConvAvg(TMAG5273_X32_CONV_AVG);
    
    float bx = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      bx += tmag.getXData();
      delay(10);
    }
    tmagBaselineX = bx / BASELINE_SAMPLES;
    Serial.println("[OK] TMAG5273 Initialized and Baselined.");
  } else {
    Serial.println("[FAIL] TMAG5273 Not Found!");
  }
  
  Serial.println("----------------------------------------\n");
}

// =============================================================
//  LOOP
// =============================================================
void loop() {

  // ===========================================================
  // 1. MLX Slider Logic (Right Slider - C/D)
  // ===========================================================
  if (mlxReady) {
    float x, y, z;
    if (!mlx.readData(&x, &y, &z)) {
      mlxFailCount++;
      if (mlxFailCount >= MLX_FAIL_LIMIT) {
        if (millis() - mlxLastFailMs >= MLX_RECOVERY_MS) {
          mlxLastFailMs = millis();
          Serial.println("[MLX] Soft recovery attempt...");
          if (mlxSoftRecover()) {
            mlxFailCount = 0;
            mlxSliderState  = 0;
            Serial.println("[MLX] Soft recovery OK.");
          }
        }
      }
    } else {
      mlxFailCount = 0;
      float dx = x - mlxBaselineX; 

      // Print raw value changes so you can see where to set thresholds
      if (abs(dx - lastMlxDebugDx) > 30.0f) {
          Serial.printf("MLX Raw DX: %8.1f\n", dx);
          lastMlxDebugDx = dx;
      }

      if (millis() - lastMlxTimeMs >= SLIDER_COOLDOWN_MS) {
        if (mlxSliderState == 0) {
          if (dx > MLX_UP_THRESHOLD) {
            mlxSliderState = 1;
            lastMlxTimeMs = millis();
            Serial.printf(">> [TRIGGER] MLX UP   -> 'c' (Value: %.1f)\n", dx);
          } else if (dx < MLX_DOWN_THRESHOLD) {
            mlxSliderState = -1;
            lastMlxTimeMs = millis();
            Serial.printf(">> [TRIGGER] MLX DOWN -> 'd' (Value: %.1f)\n", dx);
          }
        } else if (mlxSliderState == 1 && dx < MLX_DEADZONE) {
          mlxSliderState = 0; 
          lastMlxTimeMs = millis();
          Serial.println("<< MLX Reset to CENTER (from UP)");
        } else if (mlxSliderState == -1 && dx > -MLX_DEADZONE) {
          mlxSliderState = 0; 
          lastMlxTimeMs = millis();
          Serial.println("<< MLX Reset to CENTER (from DOWN)");
        }
      }
    }
  }

  // ===========================================================
  // 2. TMAG Slider Logic (Left Slider - A/B)
  // ===========================================================
  if (tmagReady && (millis() - lastTmagReadMs >= TMAG_INTERVAL_MS)) {
    lastTmagReadMs = millis();
    
    // Read the primary axis of movement
    float currentX = tmag.getXData();
    float tdx = currentX - tmagBaselineX;

    // Print raw value changes so you can see where to set thresholds
    if (abs(tdx - lastTmagDebugDx) > 0.3f) {
      Serial.printf("TMAG Raw DX: %8.2f mT\n", tdx);
      lastTmagDebugDx = tdx;
    }

    if (millis() - lastTmagTimeMs >= SLIDER_COOLDOWN_MS) {
      if (tmagSliderState == 0) {
        if (tdx > TMAG_UP_THRESHOLD) {
          tmagSliderState = 1;
          lastTmagTimeMs = millis();
          Serial.printf(">> [TRIGGER] TMAG UP   -> 'a' (Value: %.2f mT)\n", tdx);
        } else if (tdx < TMAG_DOWN_THRESHOLD) {
          tmagSliderState = -1;
          lastTmagTimeMs = millis();
          Serial.printf(">> [TRIGGER] TMAG DOWN -> 'b' (Value: %.2f mT)\n", tdx);
        }
      } else if (tmagSliderState == 1 && tdx < TMAG_DEADZONE) {
        tmagSliderState = 0; 
        lastTmagTimeMs = millis();
        Serial.println("<< TMAG Reset to CENTER (from UP)");
      } else if (tmagSliderState == -1 && tdx > -TMAG_DEADZONE) {
        tmagSliderState = 0; 
        lastTmagTimeMs = millis();
        Serial.println("<< TMAG Reset to CENTER (from DOWN)");
      }
    }
  }
}
