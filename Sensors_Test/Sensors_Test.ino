#include <TRSensors.h>

// AlphaBot2-Ar are 5 senzori IR
TRSensors trs = TRSensors();

unsigned int sensorValues[5];  // valorile brute: 0 (alb) ~ 1000 (negru)
unsigned int calibratedPos;    // pozitia liniei: 0 (stanga) ~ 4000 (dreapta)

void setup() {
  Serial.begin(9600);
  Serial.println("=== Test senzori AlphaBot2-Ar ===");
  Serial.println("Tine robotul deasupra suprafetei albe...");
  delay(2000);

  // =============================================
  // CALIBRARE — misca robotul stanga-dreapta manual
  // timp de 5 secunde deasupra traseului
  // =============================================
  Serial.println("Incepe calibrarea! Misca robotul stanga-dreapta pe linie...");

  for (int i = 0; i < 100; i++) {
    trs.calibrate();  // apeleaza des, timp de ~5 secunde
    delay(50);
  }

  Serial.println("Calibrare terminata!");
  Serial.println("Valorile min/max per senzor:");

  for (int i = 0; i < 5; i++) {
    Serial.print("  S"); Serial.print(i + 1);
    Serial.print(": min="); Serial.print(trs.calibratedMin[i]);
    Serial.print("  max="); Serial.println(trs.calibratedMax[i]);
  }

  Serial.println("\nS1=stanga | S2 | S3=centru | S4 | S5=dreapta");
  Serial.println("Valori: 0=alb, 1000=negru");
  delay(1000);
}

void loop() {
  // Citeste pozitia liniei (0-4000) + valorile brute ale fiecarui senzor
  calibratedPos = trs.readLine(sensorValues);

  // Afiseaza valorile brute
  Serial.print("Senzori: [");
  for (int i = 0; i < 5; i++) {
    // Afisam 0 sau 1 pentru claritate (prag la 500)
    Serial.print(sensorValues[i] > 500 ? "1" : "0");
    if (i < 4) Serial.print(", ");
  }
  Serial.print("]  Pozitie: ");
  Serial.print(calibratedPos);

  // Afiseaza si eroarea fata de centru (pentru PID)
  int eroare = (int)calibratedPos - 2000;  // centru = 2000, range: -2000 la +2000
  Serial.print("  Eroare: ");
  Serial.println(eroare);

  delay(100);
}