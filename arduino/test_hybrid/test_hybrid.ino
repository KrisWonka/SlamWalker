// Hybrid test: motors run from hardcoded value + report odom over serial
// If motors spin: serial + motor control can coexist
// If motors DON'T spin: USB electrical issue confirmed

const int enA = 11, in1 = 10, in2 = 9;
const int enB = 5, in3 = 8, in4 = 12;
const int ENC_L_A = 2, ENC_R_A = 3, ENC_L_B = 4, ENC_R_B = 7;

volatile long encTicksL = 0, encTicksR = 0;
long prevL = 0, prevR = 0;
unsigned long lastOdom = 0;

void isrL() { encTicksL += (digitalRead(ENC_L_B)==digitalRead(ENC_L_A)) ? 1 : -1; }
void isrR() { encTicksR += (digitalRead(ENC_R_B)==digitalRead(ENC_R_A)) ? 1 : -1; }

void setup() {
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
  Serial.begin(115200);

  // Drive motors directly - NO serial needed
  analogWrite(enA, 150);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  analogWrite(enB, 150);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  lastOdom = millis();
  Serial.println("HYBRID_TEST_RUNNING");
}

void loop() {
  if (millis() - lastOdom >= 50) {
    noInterrupts();
    long cL = encTicksL, cR = encTicksR;
    interrupts();
    long dL = cL - prevL, dR = cR - prevR;
    prevL = cL; prevR = cR;
    unsigned long dt = millis() - lastOdom;
    lastOdom = millis();
    Serial.print("O"); Serial.print(dL);
    Serial.print(","); Serial.print(dR);
    Serial.print(","); Serial.println(dt);
  }
}
