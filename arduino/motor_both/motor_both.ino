// MD10 both-motor test — drives BOTH motors FORWARD on power-up.
// Pinout matches the current production firmware (test.ino): 2x Cytron
// SHIELD-MD10 R2, single channel each:  LEFT PWM=9/DIR=8, RIGHT PWM=11/DIR=12.
// DIR LOW = forward (MD10 truth table).
//
// No serial commands needed — this isolates the motor/driver/power chain
// from the Jetson serial link. If a side doesn't spin, that side's
// motor / MD10 / wiring is the fault.

const int LEFT_PWM  = 9;
const int LEFT_DIR  = 8;
const int RIGHT_PWM = 11;
const int RIGHT_DIR = 12;

const int PWM_LEVEL = 200;   // ~78%, same level as the old motor_test sketch

void setup() {
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("MD10 BOTH-MOTOR TEST - both should spin FORWARD now");

  digitalWrite(LEFT_DIR,  LOW);   // forward
  digitalWrite(RIGHT_DIR, LOW);   // forward
  analogWrite(LEFT_PWM,  PWM_LEVEL);
  analogWrite(RIGHT_PWM, PWM_LEVEL);

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  delay(500);
  Serial.println("running L+R fwd...");
}
