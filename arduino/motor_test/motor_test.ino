// Minimal motor test - drives both motors on power up
// No serial commands needed. If motors don't spin, wiring is wrong.

const int enA = 11;
const int in1 = 10;
const int in2 = 9;
const int enB = 5;
const int in3 = 8;
const int in4 = 12;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("MOTOR TEST - both motors should spin now!");

  // Left motor forward full speed
  analogWrite(enA, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Right motor forward full speed
  analogWrite(enB, 200);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // Keep running, blink LED
  delay(500);
  Serial.println("Running...");
}
