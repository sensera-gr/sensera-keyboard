#include <Arduino.h>

// --- HARDWARE PINS ---
const int   HALL_PIN_A  = 1; // Top sensor
const int   HALL_PIN_B  = 2; // Bottom sensor
const char  KEY_UP      = 'a';
const char  KEY_DOWN    = 'b';

#define ADC_RESOLUTION   12
#define ADC_ATTENUATION  ADC_11db
#define HYPER_SAMPLES    128

// --- CALIBRATION THRESHOLDS ---
// Raised significantly to ignore ESP32 ADC noise!
const float DRV_UP_THRESHOLD   = 35.0f;  
const float DRV_DOWN_THRESHOLD = -35.0f; 
const float DRV_DEADZONE       = 15.0f;  

float baselineA = 0;
float baselineB = 0;

int           drvSliderState   = 0; 
float         lastDrvDebugDx   = 0; 
unsigned long lastDrvTimeMs    = 0;
#define SLIDER_COOLDOWN_MS 150 
#define HALL_INTERVAL_MS 20
unsigned long lastHallMs = 0;

// =============================================================
//  ADC HELPER
// =============================================================
float readADC_HighRes(int pin) {
  long sum = 0;
  for (int i = 0; i < HYPER_SAMPLES; i++) sum += analogRead(pin);
  return (float)sum / (float)HYPER_SAMPLES;
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n--- DRV5055 Virtual Axis Calibration Mode ---");
  Serial.println("IMPORTANT: Ensure slider is CENTERED right now!");
  delay(1000);

  // Analog Hall Init
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);
  pinMode(HALL_PIN_A, INPUT);
  pinMode(HALL_PIN_B, INPUT);

  // Capture baseline exactly ONCE. No auto-calibration later.
  baselineA = readADC_HighRes(HALL_PIN_A);
  baselineB = readADC_HighRes(HALL_PIN_B);

  Serial.printf("[OK] Baseline Locked -> Hall A: %.1f | Hall B: %.1f\n", baselineA, baselineB);
  Serial.println("----------------------------------------\n");
}

// =============================================================
//  LOOP
// =============================================================
void loop() {
  if (millis() - lastHallMs >= HALL_INTERVAL_MS) {
    lastHallMs = millis();

    // 1. Read current raw values
    float currentA = readADC_HighRes(HALL_PIN_A);
    float currentB = readADC_HighRes(HALL_PIN_B);

    // 2. Calculate deviation from the locked baseline
    float devA = currentA - baselineA;
    float devB = currentB - baselineB;

    // 3. Create the Virtual Axis (Differential)
    // If magnet approaches A, devA increases and devB decreases -> Highly Positive
    // If magnet approaches B, devB increases and devA decreases -> Highly Negative
    float virtualAxis = devA - devB; 

    // Print raw values to Serial Monitor so you can tune the thresholds
    if (abs(virtualAxis - lastDrvDebugDx) > 3.0f) {
        Serial.printf("DRV Virtual Axis: %8.1f\n", virtualAxis);
        lastDrvDebugDx = virtualAxis;
    }

    // 4. State Machine Trigger Logic
    if (millis() - lastDrvTimeMs >= SLIDER_COOLDOWN_MS) {
      if (drvSliderState == 0) {
        if (virtualAxis > DRV_UP_THRESHOLD) {
          drvSliderState = 1;
          lastDrvTimeMs = millis();
          Serial.printf(">> [TRIGGER] Slider UP   -> '%c' (Value: %.1f)\n", KEY_UP, virtualAxis);
        } else if (virtualAxis < DRV_DOWN_THRESHOLD) {
          drvSliderState = -1;
          lastDrvTimeMs = millis();
          Serial.printf(">> [TRIGGER] Slider DOWN -> '%c' (Value: %.1f)\n", KEY_DOWN, virtualAxis);
        }
      } else if (drvSliderState == 1 && virtualAxis < DRV_DEADZONE) {
        drvSliderState = 0; 
        lastDrvTimeMs = millis();
        Serial.println("<< [RESET] Slider returned to CENTER (from UP)");
      } else if (drvSliderState == -1 && virtualAxis > -DRV_DEADZONE) {
        drvSliderState = 0; 
        lastDrvTimeMs = millis();
        Serial.println("<< [RESET] Slider returned to CENTER (from DOWN)");
      }
    }
  }
}
