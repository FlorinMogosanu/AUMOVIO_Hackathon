// ================================================================
//  AlphaBot2-AR — Line Maze Solver  (Left-Hand Rule + PID)
//
//  Rezolva labirinte cu: linii drepte, colturi 90°, T-junctii,
//  intersectii in cruce, fundaturi, curbe, hexagoane
//
//  Algoritm la junctie:  stanga > drept > dreapta > intoarcere 180°
//
//  Flow:  Buton → Calibrare automata → Centrare pe linie → GO
// ================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <TRSensors.h>

// ── Pini motoare ─────────────────────────────────────────────────
#define PWMA  6
#define AIN1  A1
#define AIN2  A0
#define PWMB  5
#define BIN1  A2
#define BIN2  A3

// ── Periferie ────────────────────────────────────────────────────
#define NEOPIXEL_PIN  7
#define NUM_SENSORS   5
#define OLED_RESET    9
#define OLED_SA0      8
#define PCF8574_ADDR  0x20

Adafruit_SSD1306   display(OLED_RESET, OLED_SA0);
Adafruit_NeoPixel  RGB(4, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
TRSensors          trs;

unsigned int sensorValues[NUM_SENSORS];

// ================================================================
//  PARAMETRI — ajusteaza daca robotul nu urmareste bine linia
// ================================================================
float KP = 0.15f;
float KD = 1.2f;
float KI = 0.0f;

#define BASE_SPEED     60    // viteza pe linie dreapta/curba
#define TURN_SPEED     65    // viteza pivot la viraje
#define MAX_SPEED     160
#define MIN_SPEED     -70    // permite pivot activ pe colturi

// ── Praguri senzori (dupa calibrare: 0=alb, 1000=negru) ─────────
#define LINE_THRESHOLD     300   // senzor considerat "pe linie"
#define BRANCH_THRESHOLD   650   // senzor exterior confirma ramura reala
#define INTERSECT_SENSORS    4   // nr. senzori activi = zona junctie

// ── Timeout-uri ──────────────────────────────────────────────────
#define LOST_TIMEOUT      3000   // ms pana la oprire dupa pierdere linie
#define TURN_TIMEOUT      1200   // ms maxim pentru un viraj (safety)
#define UTURN_TIMEOUT     1800   // ms maxim pentru intoarcere 180°

// ================================================================

// PID
float  pidError    = 0;
float  lastError   = 0;
float  integralErr = 0;

// Stare
bool          running  = false;
bool          lineLost = false;
unsigned long lostTime = 0;

// Detectie ramuri inainte de junctie
bool seenLeft  = false;
bool seenRight = false;

// ── PCF8574 ──────────────────────────────────────────────────────
void PCF8574Write(byte data) {
  Wire.beginTransmission(PCF8574_ADDR);
  Wire.write(data);
  Wire.endTransmission();
}

byte PCF8574Read() {
  Wire.requestFrom(PCF8574_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool buttonPressed() {
  PCF8574Write(0x1F | PCF8574Read());
  return ((PCF8574Read() | 0xE0) == 0xEF);
}

// Senzori IR obstacol = bits 6 si 7 (ca in UltraSensor_Example)
bool obstacleDetected() {
  PCF8574Write(0xC0 | PCF8574Read());
  return ((PCF8574Read() | 0x3F) != 0xFF);
}

// ── Motoare ──────────────────────────────────────────────────────
void motorLeft(int s) {
  s = constrain(s, -MAX_SPEED, MAX_SPEED);
  if (s >= 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA,  s); }
  else        { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  analogWrite(PWMA, -s); }
}

void motorRight(int s) {
  s = constrain(s, -MAX_SPEED, MAX_SPEED);
  if (s >= 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB,  s); }
  else        { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  analogWrite(PWMB, -s); }
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void setLED(uint32_t c) {
  for (int i = 0; i < 4; i++) RGB.setPixelColor(i, c);
  RGB.show();
}

// ── Senzori ──────────────────────────────────────────────────────
int sensorsOnLine() {
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    if (sensorValues[i] > LINE_THRESHOLD) n++;
  return n;
}

// ── Display ───────────────────────────────────────────────────────
void showMessage(const char* l1, const char* l2 = "", int sz = 1) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(sz > 0 ? sz : 1);
  display.setCursor(0, 10); display.println(l1);
  if (strlen(l2) > 0) { display.setCursor(0, 35); display.println(l2); }
  display.display();
}

void showCenteringDisplay() {
  unsigned int pos = trs.readLine(sensorValues) / 200;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0, 0);  display.println("Centreaza pe linie:");
  display.setCursor(0, 20); display.println("Apasa buton -> START");
  display.setCursor(0, 45);
  for (int i = 0; i < 21; i++) display.print('_');
  display.setCursor(pos * 6, 45); display.print("||");
  display.setCursor(60, 55);      display.print("^");
  display.display();
}

// ================================================================
//  FUNCTII VIRAJ
//  Fiecare viraj:
//    1. Pivoteaza cu intarziere minima (sa paraseasca linia curenta)
//    2. Asteapta ca S2 (senzor central) sa gaseasca noua ramura
//    3. Se opreste si reseteaza starea PID
// ================================================================

void doTurnLeft() {
  setLED(0x0000FF);               // albastru = viraj stanga
  motorLeft(-TURN_SPEED);
  motorRight(TURN_SPEED);
  delay(230);                     // ~40°: paraseste zona junctiei
  trs.readLine(sensorValues);
  unsigned long t = millis();
  while (sensorValues[2] < LINE_THRESHOLD && millis() - t < TURN_TIMEOUT) {
    trs.readLine(sensorValues);
    delay(4);
  }
  stopMotors(); delay(40);
  lastError = 0; integralErr = 0;
}

void doTurnRight() {
  setLED(0x0000FF);               // albastru = viraj dreapta
  motorLeft(TURN_SPEED);
  motorRight(-TURN_SPEED);
  delay(230);
  trs.readLine(sensorValues);
  unsigned long t = millis();
  while (sensorValues[2] < LINE_THRESHOLD && millis() - t < TURN_TIMEOUT) {
    trs.readLine(sensorValues);
    delay(4);
  }
  stopMotors(); delay(40);
  lastError = 0; integralErr = 0;
}

void doUTurn() {
  setLED(0xFF00FF);               // magenta = intoarcere
  motorLeft(TURN_SPEED);
  motorRight(-TURN_SPEED);
  // Trece peste linia de intrare (~90°) si continua spre 180°
  delay(420);
  // Asteapta S2 sa gaseasca linia de retur
  trs.readLine(sensorValues);
  unsigned long t = millis();
  while (sensorValues[2] < LINE_THRESHOLD && millis() - t < UTURN_TIMEOUT) {
    trs.readLine(sensorValues);
    delay(4);
  }
  stopMotors(); delay(40);
  lastError = 0; integralErr = 0;
}

// ================================================================
//  HANDLE JUNCTION — Left-Hand Rule
//
//  Intrare: robotul este la/langa centrul junctiei (onLine >= 4)
//  1. Avanseaza 130ms pentru a trece de centrul junctiei
//  2. Citeste starea: exista cale drept? (S2)
//  3. Foloseste flags seenLeft/seenRight detectate in apropierea junctiei
//  4. Decide: stanga > drept > dreapta > intoarcere
// ================================================================
void handleJunction() {
  setLED(0xFFFF00);  // galben = procesare junctie

  // Avanseaza sa treaca centrul → poate detecta si mai sus/jos
  unsigned long t0 = millis();
  while (millis() - t0 < 130) {
    motorLeft(BASE_SPEED * 0.65);
    motorRight(BASE_SPEED * 0.65);
    trs.readLine(sensorValues);
    if (sensorValues[0] > BRANCH_THRESHOLD) seenLeft  = true;
    if (sensorValues[4] > BRANCH_THRESHOLD) seenRight = true;
    delay(5);
  }
  stopMotors(); delay(30);

  trs.readLine(sensorValues);
  bool straightOK = (sensorValues[2] > LINE_THRESHOLD);
  bool leftOK     = seenLeft;
  bool rightOK    = seenRight;

  // Fundatura: nicio cale disponibila
  bool deadEnd = (!leftOK && !straightOK && !rightOK);

  // Left-Hand Rule
  if (deadEnd)        { doUTurn();    }
  else if (leftOK)    { doTurnLeft(); }
  else if (straightOK){ lastError = 0; integralErr = 0; }  // continua drept
  else                { doTurnRight(); }

  seenLeft  = false;
  seenRight = false;
  setLED(0xFF0000);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(); RGB.begin(); setLED(0x000000);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.setTextSize(2); display.setTextColor(WHITE);
  display.setCursor(10, 0);  display.println("AlphaBot2");
  display.setTextSize(1);
  display.setCursor(5, 30);  display.println("Maze Solver");
  display.setCursor(5, 50);  display.println("Apasa pt calibrare");
  display.display(); setLED(0xFFFF00);

  // ── FAZA 1: Asteapta buton ────────────────────────────────────
  while (!buttonPressed()) delay(50);
  delay(300);

  // ── FAZA 2: Calibrare ─────────────────────────────────────────
  showMessage("Calibrare...", "Miscare S-D");
  setLED(0x00FF00);
  analogWrite(PWMA, 90); analogWrite(PWMB, 90);
  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    } else {
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    }
    trs.calibrate(); delay(80);
  }
  stopMotors(); setLED(0x0000FF);

  // ── FAZA 3: Centrare manuala ──────────────────────────────────
  while (!buttonPressed()) { showCenteringDisplay(); delay(50); }
  delay(300);

  // ── FAZA 4: GO ───────────────────────────────────────────────
  display.clearDisplay(); display.setTextSize(3); display.setCursor(30, 20);
  display.println("GO!"); display.display(); setLED(0xFF0000);

  pidError = 0; lastError = 0; integralErr = 0; lostTime = 0;
  seenLeft = false; seenRight = false;
  delay(800);
  running = true;
}

// ================================================================
//  LOOP — Maze Solver + PID
// ================================================================
void loop() {
  if (!running) return;

  // ── 0. Obstacol in fata → intoarcere 180° ────────────────────
  if (obstacleDetected()) {
    stopMotors();
    setLED(0xFFFFFF);  // alb = obstacol
    delay(200);
    doUTurn();
    setLED(0xFF0000);
    return;
  }

  unsigned int position = trs.readLine(sensorValues);
  int onLine = sensorsOnLine();

  // ── 1. Linie pierduta ─────────────────────────────────────────
  if (onLine == 0) {
    if (!lineLost) { lineLost = true; lostTime = millis(); setLED(0xFF4400); }
    if (millis() - lostTime > LOST_TIMEOUT) {
      stopMotors(); setLED(0xFF0000);
      showMessage("LINIE PIERDUTA", "Repozitioneaza!");
      running = false; return;
    }
    int srch = 70;
    if (lastError < 0) { motorLeft(-srch); motorRight(srch); }
    else               { motorLeft(srch);  motorRight(-srch); }
    return;
  }
  if (lineLost) { lineLost = false; lostTime = 0; setLED(0xFF0000); }

  // ── 2. Pre-detectie ramuri laterale ───────────────────────────
  //  Senzor exterior (S0 sau S4) cu prag ridicat (650) = ramura reala,
  //  nu marginea liniei curente. Se verifica si ca robotul e centrat
  //  (eroarea mica) ca sa nu confundam o deviatie cu o ramura.
  float currentError = (float)position - 2000.0f;
  if (sensorValues[0] > BRANCH_THRESHOLD && currentError > -600) seenLeft  = true;
  if (sensorValues[4] > BRANCH_THRESHOLD && currentError <  600) seenRight = true;

  // ── 3. Junctie (>=4 senzori pe negru) ─────────────────────────
  if (onLine >= INTERSECT_SENSORS) {
    handleJunction();
    return;
  }

  // ── 4. PID normal ─────────────────────────────────────────────
  pidError = currentError;
  integralErr = constrain(integralErr + pidError, -3000.0f, 3000.0f);
  float deriv = pidError - lastError;
  lastError = pidError;
  float output = (KP * pidError) + (KI * integralErr) + (KD * deriv);

  int speedLeft  = constrain((int)(BASE_SPEED + output), MIN_SPEED, MAX_SPEED);
  int speedRight = constrain((int)(BASE_SPEED - output), MIN_SPEED, MAX_SPEED);
  motorLeft(speedLeft);
  motorRight(speedRight);

  // Debug Serial — decomenta pentru tuning:
  // Serial.print("P:"); Serial.print(position);
  // Serial.print(" L:"); Serial.print(seenLeft);
  // Serial.print(" R:"); Serial.println(seenRight);
}