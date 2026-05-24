// Quick encoder pin diagnostic
// Upload this, open serial monitor at 115200, spin wheels by hand
// Watch if pin states change - if they don't, wires are disconnected

const int ENC_L_A = 2;
const int ENC_R_A = 3;
const int ENC_L_B = 4;
const int ENC_R_B = 7;

volatile long ticksL = 0;
volatile long ticksR = 0;

void isrL() { ticksL++; }
void isrR() { ticksR++; }

void setup() {
  Serial.begin(115200);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
  Serial.println("ENCODER DIAG - spin wheels by hand");
}

void loop() {
  int la = digitalRead(ENC_L_A);
  int lb = digitalRead(ENC_L_B);
  int ra = digitalRead(ENC_R_A);
  int rb = digitalRead(ENC_R_B);

  Serial.print("L_A="); Serial.print(la);
  Serial.print(" L_B="); Serial.print(lb);
  Serial.print(" R_A="); Serial.print(ra);
  Serial.print(" R_B="); Serial.print(rb);
  Serial.print("  ticksL="); Serial.print(ticksL);
  Serial.print("  ticksR="); Serial.println(ticksR);
  delay(200);
}
