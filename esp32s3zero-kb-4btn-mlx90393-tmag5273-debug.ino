#include <Wire.h>
#include <Adafruit_MLX90393.h>
#include <SparkFun_TMAG5273_Arduino_Library.h> 

// --- HARDWARE PINS ---
#define I2C_SDA 8
#define I2C_SCL 9

#define BASELINE_SAMPLES   20
#define SLIDER_COOLDOWN_MS 150 

// =============================================================
//  MLX90393 CROSS SLIDER (Right Side: C/D/E/F)
// =============================================================
Adafruit_MLX90393 mlx = Adafruit_MLX90393();
bool  mlxReady  = false;
float mlxBaselineX = 0, mlxBaselineY = 0, mlxBaselineZ = 0;

// CALIBRATION THRESHOLDS (Raw Bits)
const float MLX_X_POS_THRES = 450.0f;  // UP (c)
const float MLX_X_NEG_THRES = -450.0f; // DOWN (d)
const float MLX_Y_POS_THRES = 450.0f;  // RIGHT (f)
const float MLX_Y_NEG_THRES = -450.0f; // LEFT (e)
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
//  TMAG5273 CROSS SLIDER (Left Side: A/B/L/R)
// =============================================================
TMAG5273 tmag;
bool  tmagReady = false;
float tmagBaselineX = 0; 
float tmagBaselineY = 0; 

// CALIBRATION THRESHOLDS (Milliteslas - mT)
const float TMAG_X_POS_THRES = 2.0f;  // UP (a)
const float TMAG_X_NEG_THRES = -2.0f; // DOWN (b)
const float TMAG_Y_POS_THRES = 2.0f;  // RIGHT (r)
const float TMAG_Y_NEG_THRES = -2.0f; // LEFT (l)
const float TMAG_DEADZONE    = 0.5f;  

int           tmagStateX      = 0;
int           tmagStateY      = 0;
float         lastTmagDebugX  = 0;
float         lastTmagDebugY  = 0;
unsigned long lastTmagTimeX   = 0;
unsigned long lastTmagTimeY   = 0;

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
  delay(2000); 
  Serial.println("\n--- 4-Way Cross Sensor Calibration Mode ---");

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
    tmag.setConvAvg(TMAG5273_X32_CONVERSION); 
    
    float bx = 0, by = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
      bx += tmag.getXData();
      by += tmag.getYData();
      delay(10);
    }
    tmagBaselineX = bx / BASELINE_SAMPLES;
    tmagBaselineY = by / BASELINE_SAMPLES;
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
  // 1. MLX CROSS LOGIC (Right Side)
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
            mlxStateX = 0;
            mlxStateY = 0;
            Serial.println("[MLX] Soft recovery OK.");
          }
        }
      }
    } else {
      mlxFailCount = 0;
      float dx = x - mlxBaselineX; 
      float dy = y - mlxBaselineY; 

      // Print raw value changes so you can see where to set thresholds
      if (abs(dx - lastMlxDebugX) > 30.0f) {
          Serial.printf("MLX Raw DX: %8.1f\n", dx);
          lastMlxDebugX = dx;
      }
      if (abs(dy - lastMlxDebugY) > 30.0f) {
          Serial.printf("MLX Raw DY: %8.1f\n", dy);
          lastMlxDebugY = dy;
      }

      // --- MLX X-Axis (Up/Down) ---
      if (millis() - lastMlxTimeX >= SLIDER_COOLDOWN_MS) {
        if (mlxStateX == 0) {
          if (dx > MLX_X_POS_THRES) {
            mlxStateX = 1; lastMlxTimeX = millis();
            Serial.printf(">> [TRIGGER] MLX UP    -> 'c' (DX: %.1f)\n", dx);
          } else if (dx < MLX_X_NEG_THRES) {
            mlxStateX = -1; lastMlxTimeX = millis();
            Serial.printf(">> [TRIGGER] MLX DOWN  -> 'd' (DX: %.1f)\n", dx);
          }
        } else if (mlxStateX == 1 && dx < MLX_DEADZONE) {
          mlxStateX = 0; lastMlxTimeX = millis(); Serial.println("<< MLX Reset X (from UP)");
        } else if (mlxStateX == -1 && dx > -MLX_DEADZONE) {
          mlxStateX = 0; lastMlxTimeX = millis(); Serial.println("<< MLX Reset X (from DOWN)");
        }
      }

      // --- MLX Y-Axis (Left/Right) ---
      if (millis() - lastMlxTimeY >= SLIDER_COOLDOWN_MS) {
        if (mlxStateY == 0) {
          if (dy > MLX_Y_POS_THRES) {
            mlxStateY = 1; lastMlxTimeY = millis();
            Serial.printf(">> [TRIGGER] MLX RIGHT -> 'f' (DY: %.1f)\n", dy);
          } else if (dy < MLX_Y_NEG_THRES) {
            mlxStateY = -1; lastMlxTimeY = millis();
            Serial.printf(">> [TRIGGER] MLX LEFT  -> 'e' (DY: %.1f)\n", dy);
          }
        } else if (mlxStateY == 1 && dy < MLX_DEADZONE) {
          mlxStateY = 0; lastMlxTimeY = millis(); Serial.println("<< MLX Reset Y (from RIGHT)");
        } else if (mlxStateY == -1 && dy > -MLX_DEADZONE) {
          mlxStateY = 0; lastMlxTimeY = millis(); Serial.println("<< MLX Reset Y (from LEFT)");
        }
      }
    }
  }

  // ===========================================================
  // 2. TMAG CROSS LOGIC (Left Side)
  // ===========================================================
  if (tmagReady && (millis() - lastTmagReadMs >= TMAG_INTERVAL_MS)) {
    lastTmagReadMs = millis();
    
    float currentX = tmag.getXData();
    float currentY = tmag.getYData();
    float tdx = currentX - tmagBaselineX;
    float tdy = currentY - tmagBaselineY;

    // Print raw value changes so you can see where to set thresholds
    if (abs(tdx - lastTmagDebugX) > 0.3f) {
      Serial.printf("TMAG Raw DX: %8.2f mT\n", tdx);
      lastTmagDebugX = tdx;
    }
    if (abs(tdy - lastTmagDebugY) > 0.3f) {
      Serial.printf("TMAG Raw DY: %8.2f mT\n", tdy);
      lastTmagDebugY = tdy;
    }

    // --- TMAG X-Axis (Up/Down) ---
    if (millis() - lastTmagTimeX >= SLIDER_COOLDOWN_MS) {
      if (tmagStateX == 0) {
        if (tdx > TMAG_X_POS_THRES) {
          tmagStateX = 1; lastTmagTimeX = millis();
          Serial.printf(">> [TRIGGER] TMAG UP    -> 'a' (DX: %.2f mT)\n", tdx);
        } else if (tdx < TMAG_X_NEG_THRES) {
          tmagStateX = -1; lastTmagTimeX = millis();
          Serial.printf(">> [TRIGGER] TMAG DOWN  -> 'b' (DX: %.2f mT)\n", tdx);
        }
      } else if (tmagStateX == 1 && tdx < TMAG_DEADZONE) {
        tmagStateX = 0; lastTmagTimeX = millis(); Serial.println("<< TMAG Reset X (from UP)");
      } else if (tmagStateX == -1 && tdx > -TMAG_DEADZONE) {
        tmagStateX = 0; lastTmagTimeX = millis(); Serial.println("<< TMAG Reset X (from DOWN)");
      }
    }

    // --- TMAG Y-Axis (Left/Right) ---
    if (millis() - lastTmagTimeY >= SLIDER_COOLDOWN_MS) {
      if (tmagStateY == 0) {
        if (tdy > TMAG_Y_POS_THRES) {
          tmagStateY = 1; lastTmagTimeY = millis();
          Serial.printf(">> [TRIGGER] TMAG RIGHT -> 'r' (DY: %.2f mT)\n", tdy);
        } else if (tdy < TMAG_Y_NEG_THRES) {
          tmagStateY = -1; lastTmagTimeY = millis();
          Serial.printf(">> [TRIGGER] TMAG LEFT  -> 'l' (DY: %.2f mT)\n", tdy);
        }
      } else if (tmagStateY == 1 && tdy < TMAG_DEADZONE) {
        tmagStateY = 0; lastTmagTimeY = millis(); Serial.println("<< TMAG Reset Y (from RIGHT)");
      } else if (tmagStateY == -1 && tdy > -TMAG_DEADZONE) {
        tmagStateY = 0; lastTmagTimeY = millis(); Serial.println("<< TMAG Reset Y (from LEFT)");
      }
    }
  }
}
