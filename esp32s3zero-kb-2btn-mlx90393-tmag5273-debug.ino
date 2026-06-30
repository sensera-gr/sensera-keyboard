#include <Wire.h>
#include <SparkFun_TMAG5273_Arduino_Library.h>
 
// Δημιουργία του αντικειμένου για τον αισθητήρα
TMAG5273 sensor;
 
// --- HARDWARE PINS ---
#define I2C_SDA 8
#define I2C_SCL 9
 
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 
  Serial.println("Εκκίνηση I2C...");
  // 1. Ορισμός των custom I2C pins για το ESP32-S3 Zero
  Wire.begin(I2C_SDA, I2C_SCL);
 
  // 2. Αρχικοποίηση του αισθητήρα. Περνάμε τη διεύθυνση (0x35) 
  // και το αντικείμενο Wire που μόλις ρυθμίσαμε.
  // Σημείωση: Αν έχεις την έκδοση A2 του αισθητήρα, άλλαξε το 0x35 σε 0x22
  if (sensor.begin(0x35, Wire) == false) {
    Serial.println("Σφάλμα: Ο TMAG5273 δεν βρέθηκε στο I2C bus!");
    while (1); // Κολλάει εδώ αν υπάρχει σφάλμα
  }
  Serial.println("Ο TMAG5273 βρέθηκε και ρυθμίστηκε με επιτυχία!\n");
}
 
void loop() {
  // Η βιβλιοθήκη SparkFun επιστρέφει απευθείας τα mT σε δεκαδική μορφή
  float x_mT = sensor.getXData();
  float y_mT = sensor.getYData();
  float z_mT = sensor.getZData();
 
  // Υπολογισμός της αρχικής (Raw 16-bit) τιμής από τα mT
  // Ο τύπος της βιβλιοθήκης είναι: mT = (Raw / 32768.0) * 40.0
  // Λύνοντας ως προς Raw έχουμε την παρακάτω πράξη:
  int16_t x_raw = (x_mT / 40.0) * 32768;
  int16_t y_raw = (y_mT / 40.0) * 32768;
  int16_t z_raw = (z_mT / 40.0) * 32768;
 
  // Εκτύπωση στο Serial Monitor (Int -> mT)
  Serial.print("X: "); 
  Serial.print(x_raw); 
  Serial.print(" ("); Serial.print(x_mT, 2); Serial.print(" mT) | ");
  Serial.print("Y: "); 
  Serial.print(y_raw); 
  Serial.print(" ("); Serial.print(y_mT, 2); Serial.print(" mT) | ");
  Serial.print("Z: "); 
  Serial.print(z_raw); 
  Serial.print(" ("); Serial.print(z_mT, 2); Serial.println(" mT)");
 
  // Αναμονή 1 δευτερόλεπτο
  delay(1000);
}
