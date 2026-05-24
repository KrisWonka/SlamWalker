// PWM dead-zone finder: ramp up PWM from 80 to 255 in steps of 10

const int ENC_L_A = 2;
const int ENC_R_A = 3;
const int ENC_L_B = 4;
const int ENC_R_B = 7;

const int enA = 11;
const int in1 = 12;
const int in2 = 9;
const int enB = 5;
const int in3 = 6;
const int in4 = 8;

volatile long ticksL = 0;
volatile long ticksR = 0;

void isrL() {
  if (digitalRead(ENC_L_B) == digitalRead(ENC_L_A))
    ticksL--;
  else
    ticksL++;
}

void isrR() {
  if (digitalRead(ENC_R_B) == digitalRead(ENC_R_A))
    ticksR++;
  else
    ticksR--;
}

void stopMotors() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);

  stopMotors();
  delay(500);

  Serial.println("=== PWM DEAD-ZONE FINDER (80-255) ===");
  Serial.println("PWM\ttL\ttR\tL?\tR?");
  delay(1000);

  for (int pwm = 80; pwm <= 255; pwm += 10) {
    noInterrupts();
    ticksL = 0;
    ticksR = 0;
    interrupts();

    analogWrite(enA, pwm);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enB, pwm);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    delay(1500);
    stopMotors();
    delay(500);

    noInterrupts();
    long tL = ticksL;
    long tR = ticksR;
    interrupts();

    Serial.print(pwm);
    Serial.print("\t");
    Serial.print(tL);
    Serial.print("\t");
    Serial.print(tR);
    Serial.print("\t");
    Serial.print(abs(tL) > 20 ? "YES" : "no");
    Serial.print("\t");
    Serial.println(abs(tR) > 20 ? "YES" : "no");
  }

  Serial.println("\n=== DONE ===");
}

void loop() {}
