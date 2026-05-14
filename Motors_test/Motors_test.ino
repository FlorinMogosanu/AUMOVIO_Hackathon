// =============================================
// PINII MOTOARELOR — AlphaBot2-Ar
// =============================================
#define PWMA 6   // Left motor speed (PWM)
#define AIN1 A1   // Left motor direction 1
#define AIN2 A0   // Left motor direction 2
#define PWMB 5   // Right motor speed (PWM)
#define BIN1 A2   // Right motor direction 1
#define BIN2 A3  // Right motor direction 2


void setup() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);


  Serial.begin(115200);
  Serial.println("Test motoare — urmareste comportamentul");

  runTests();
}

void loop() {}  // toate testele ruleaza o singura data in setup

// =============================================
// FUNCTIA DE CONTROL MOTOARE
// =============================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Motor STANGA
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, constrain(leftSpeed, 0, 255));

  // Motor DREAPTA
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// =============================================
// SECVENTA DE TESTE
// =============================================
void runTests() {
  // --- TEST 1: Inainte ---
  Serial.println("\nTest 1: INAINTE — ambele roti trebuie sa mearga inainte");
  setMotors(150, 150);
  delay(1500);
  stopMotors();
  delay(1000);

  // --- TEST 2: Inapoi ---
  Serial.println("Test 2: INAPOI — ambele roti trebuie sa mearga inapoi");
  setMotors(-150, -150);
  delay(1500);
  stopMotors();
  delay(1000);

  // --- TEST 3: Viraj stanga ---
  Serial.println("Test 3: VIRAJ STANGA — stanga sta, dreapta merge");
  setMotors(0, 150);
  delay(1500);
  stopMotors();
  delay(1000);

  // --- TEST 4: Viraj dreapta ---
  Serial.println("Test 4: VIRAJ DREAPTA — dreapta sta, stanga merge");
  setMotors(150, 0);
  delay(1500);
  stopMotors();
  delay(1000);

  // --- TEST 5: Rotatie pe loc stanga ---
  Serial.println("Test 5: ROTATIE PE LOC STANGA");
  setMotors(-150, 150);
  delay(1000);
  stopMotors();
  delay(1000);

  // --- TEST 6: Rotatie pe loc dreapta ---
  Serial.println("Test 6: ROTATIE PE LOC DREAPTA");
  setMotors(150, -150);
  delay(1000);
  stopMotors();

  Serial.println("\nGata! Verifica rezultatele de mai sus.");
}