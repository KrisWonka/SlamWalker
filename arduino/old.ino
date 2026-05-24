// =============================================================
// SlamWalker Arduino Firmware  (with encoder odometry)
//
// Motor:   Pololu 37D 12V with 64 CPR encoder (#4750 family)
// Driver:  HW-095 (L298N)
// Comm:    UART 115200
//
// Serial protocol:
//   RX: "V<linear_x>,<angular_z>#\n"  velocity command (# = end marker)
//   TX: "O<dTickL>,<dTickR>,<dt_ms>\n" encoder delta (20 Hz)
//       "READY\n" at boot
// =============================================================

// -------------------- Pin Definitions -----------------------
const int ENC_L_A = 2;   // Left encoder A  (INT0)
const int ENC_R_A = 3;   // Right encoder A (INT1)
const int ENC_L_B = 4;   // Left encoder B  (digital read)
const int ENC_R_B = 7;   // Right encoder B (digital read)

// Left Motor  (HW-095 / L298N)
const int enA = 11;  // PWM
const int in1 = 12;  // was 10, that pin died
const int in2 = 9;
// Right Motor — swapped in3/in4 to fix direction
const int enB = 5;   // PWM (moved from 3, which is now ENC_R_A)
const int in3 = 6;   // was 12, that pin died
const int in4 = 8;

// ------------------- Robot Parameters -----------------------
const float WHEEL_BASE    = 0.22;   // wheel-to-wheel distance (m)
const float MAX_SPEED     = 0.60;   // max wheel linear speed (m/s)
const int   PWM_MIN_MOVE  = 70;     // dead-zone threshold (raised for loaded wheels)

const float GEAR_RATIO    = 70.0;   // Pololu 37D 70:1 (#4754)
const float COUNTS_PER_WHEEL_REV = 32.0 * GEAR_RATIO;

// Motor speed calibration: scale PWM for each motor (0.0 ~ 1.0)
// Adjust these to balance left/right speed difference
const float LEFT_MOTOR_SCALE  = 1.00;  // reduce if left is too fast
const float RIGHT_MOTOR_SCALE = 1.00;  // reduce if right is too fast

// ------------------- Safety ---------------------------------
const unsigned long CMD_TIMEOUT_MS = 500;

// ------------------- Timing ---------------------------------
const unsigned long ODOM_INTERVAL_MS = 50;  // 20 Hz odom report

// ------------------- State ----------------------------------
float targetLinear  = 0.0;
float targetAngular = 0.0;
unsigned long lastCmdTime  = 0;
unsigned long lastOdomTime = 0;
unsigned long lastDebugTime = 0;
int cmdParsedCount = 0;
int cmdRejectedCount = 0;

volatile long encTicksL = 0;
volatile long encTicksR = 0;
long prevTicksL = 0;
long prevTicksR = 0;

String inputBuffer = "";

// -------------------- Encoder ISRs --------------------------

void isrLeftA() {
  if (digitalRead(ENC_L_B) == digitalRead(ENC_L_A))
    encTicksL++;
  else
    encTicksL--;
}

void isrRightA() {
  if (digitalRead(ENC_R_B) == digitalRead(ENC_R_A))
    encTicksR++;
  else
    encTicksR--;
}

// -------------------- Motor Helpers -------------------------

void setMotor(int enPin, int fwdPin, int revPin, float speed) {
  int pwm = (int)(fabs(speed) * 255.0);
  if (pwm > 255) pwm = 255;

  if (pwm < PWM_MIN_MOVE) {
    analogWrite(enPin, 0);
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, LOW);
    return;
  }

  analogWrite(enPin, pwm);
  if (speed > 0) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(revPin, LOW);
  } else {
    digitalWrite(fwdPin, LOW);
    digitalWrite(revPin, HIGH);
  }
}

void driveMotors(float linear, float angular) {
  float vLeft  = linear - (angular * WHEEL_BASE / 2.0);
  float vRight = linear + (angular * WHEEL_BASE / 2.0);

  float leftNorm  = constrain(vLeft  / MAX_SPEED, -1.0, 1.0);
  float rightNorm = constrain(vRight / MAX_SPEED, -1.0, 1.0);

  setMotor(enA, in1, in2, leftNorm * LEFT_MOTOR_SCALE);
  setMotor(enB, in3, in4, rightNorm * RIGHT_MOTOR_SCALE);
}

void stopMotors() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// -------------------- Serial Parsing ------------------------

void parseCommand(const String &cmd) {
  if (cmd.length() < 4 || cmd.charAt(0) != 'V') {
    cmdRejectedCount++;
    return;
  }

  String payload;
  int hashIdx = cmd.indexOf('#');
  if (hashIdx > 0) {
    payload = cmd.substring(1, hashIdx);
  } else {
    payload = cmd.substring(1);
  }

  int commaIdx = payload.indexOf(',');
  if (commaIdx < 1) {
    cmdRejectedCount++;
    return;
  }

  for (unsigned int i = 0; i < payload.length(); i++) {
    char ch = payload.charAt(i);
    if (ch != '.' && ch != ',' && ch != '-' && (ch < '0' || ch > '9')) {
      cmdRejectedCount++;
      return;
    }
  }

  float lin = payload.substring(0, commaIdx).toFloat();
  float ang = payload.substring(commaIdx + 1).toFloat();

  if (fabs(lin) > 2.0 || fabs(ang) > 10.0) {
    cmdRejectedCount++;
    return;
  }

  targetLinear  = lin;
  targetAngular = ang;
  lastCmdTime   = millis();
  cmdParsedCount++;
}

// -------------------- Odom Report ---------------------------

void reportOdom() {
  noInterrupts();
  long curL = encTicksL;
  long curR = encTicksR;
  interrupts();

  long dL = curL - prevTicksL;
  long dR = curR - prevTicksR;
  prevTicksL = curL;
  prevTicksR = curR;

  unsigned long now = millis();
  unsigned long dt = now - lastOdomTime;
  lastOdomTime = now;

  Serial.print("O");
  Serial.print(dL);
  Serial.print(",");
  Serial.print(dR);
  Serial.print(",");
  Serial.println(dt);
}

// -------------------- Setup / Loop --------------------------

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Built-in LED: ON when motors are being driven
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);

  Serial.begin(115200);
  stopMotors();
  lastOdomTime = millis();
  Serial.println("READY");
}

void loop() {
  // ---- Read serial commands ----
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // ---- Safety timeout ----
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopMotors();
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    driveMotors(targetLinear, targetAngular);
    digitalWrite(LED_BUILTIN, HIGH);  // LED ON = motors active
  }

  // ---- Periodic odom report ----
  if (millis() - lastOdomTime >= ODOM_INTERVAL_MS) {
    reportOdom();
  }

  // ---- Debug: print state once per second ----
  if (millis() - lastDebugTime >= 1000) {
    lastDebugTime = millis();
    float vL = targetLinear - (targetAngular * WHEEL_BASE / 2.0);
    float vR = targetLinear + (targetAngular * WHEEL_BASE / 2.0);
    int pwmL = (int)(fabs(constrain(vL/MAX_SPEED,-1,1) * LEFT_MOTOR_SCALE) * 255);
    int pwmR = (int)(fabs(constrain(vR/MAX_SPEED,-1,1) * RIGHT_MOTOR_SCALE) * 255);
    bool timeout = (millis() - lastCmdTime > CMD_TIMEOUT_MS);
    Serial.print("D lin="); Serial.print(targetLinear, 2);
    Serial.print(" ang="); Serial.print(targetAngular, 2);
    Serial.print(" vL="); Serial.print(vL, 3);
    Serial.print(" vR="); Serial.print(vR, 3);
    Serial.print(" pwmL="); Serial.print(pwmL);
    Serial.print(" pwmR="); Serial.print(pwmR);
    Serial.print(" ok="); Serial.print(cmdParsedCount);
    Serial.print(" rej="); Serial.print(cmdRejectedCount);
    Serial.print(" tout="); Serial.println(timeout);
  }
}
