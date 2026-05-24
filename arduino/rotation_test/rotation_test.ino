// Minimal rotation test — no serial parsing, just hard-coded pin states.
// Left motor REVERSE, right motor FORWARD, both full power.

const int enA = 11;
const int in1 = 12;
const int in2 = 9;
const int enB = 5;
const int in3 = 6;
const int in4 = 8;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(115200);
  Serial.println("=== ROTATION TEST ===");

  // Left motor: REVERSE (in1=LOW, in2=HIGH)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 200);

  // Right motor: FORWARD (in3=HIGH, in4=LOW)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);

  Serial.println("Motors set: L=reverse R=forward PWM=200");
}

void loop() {
  delay(100);
  Serial.println("running...");
}
