// =============================================================
// SlamWalker Arduino Firmware  (with encoder odometry)
//
// Motor:   Pololu 37D 12V with 64 CPR encoder (#4750 family)
// Driver:  2x Cytron SHIELD-MD10 R2 (stacked, single channel each, 10A)
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

// 2x MD10 R2: board#1 jumpers JP5->D9 (PWM) JP6->D8 (DIR) for LEFT
//             board#2 jumpers JP5->D11 (PWM) JP6->D12 (DIR) for RIGHT
// DIR LOW = forward (per MD10 truth table; flip wires or invert here if reversed)
const int LEFT_PWM  = 9;
const int LEFT_DIR  = 8;
const int RIGHT_PWM = 11;
const int RIGHT_DIR = 12;

// ------------------- Robot Parameters -----------------------
const float WHEEL_BASE    = 0.3634;   // effective track width for walking mechanism (m)
const float MAX_SPEED     = 0.10;   // walking mechanism needs high PWM; lower = more torque
const int   PWM_MIN_MOVE  = 230;    // walking mechanism needs near-full power to overcome friction

const float GEAR_RATIO    = 70.0;   // Pololu 37D 70:1 (#4754)
const float COUNTS_PER_WHEEL_REV = 32.0 * GEAR_RATIO;

// Per-motor PWM bias: absolute offset subtracted from each side's PWM.
// Used because mechanical L/R imbalance is roughly a fixed PWM gap, not a ratio —
// a ratio scales turn commands too, starving the inner wheel of torque.
const int LEFT_PWM_BIAS  = 11;
const int RIGHT_PWM_BIAS = -10;  // MD10 swap: bumped right up to balance left-fast asymmetry

// ------------------- Smoothing --------------------------------
const float RAMP_RATE     = 4.0;    // max speed change per second (0→1 in 250ms)

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
    encTicksL--;
  else
    encTicksL++;
}

void isrRightA() {
  if (digitalRead(ENC_R_B) == digitalRead(ENC_R_A))
    encTicksR++;
  else
    encTicksR--;
}

// -------------------- Motor Helpers -------------------------

void setMotor(int pwmPin, int dirPin, float speed, int pwmBias) {
  float absSpd = fabs(speed);
  if (absSpd < 0.01) {
    analogWrite(pwmPin, 0);
    return;
  }

  // Remap [0.01 .. 1.0] → [PWM_MIN_MOVE .. 255], then subtract bias.
  // PWM_MIN_MOVE was tuned for L298N's ~2.5V dropout. MD10 has near-zero
  // dropout so motor will feel stronger at same PWM — may want to lower
  // PWM_MIN_MOVE after first teleop test if startup is too jerky.
  int pwm = PWM_MIN_MOVE + (int)(absSpd * (float)(255 - PWM_MIN_MOVE)) - pwmBias;
  pwm = constrain(pwm, PWM_MIN_MOVE, 255);

  digitalWrite(dirPin, speed > 0 ? LOW : HIGH);  // MD10: DIR=LOW -> forward
  analogWrite(pwmPin, pwm);
}

float smoothL = 0.0;
float smoothR = 0.0;
unsigned long lastDriveUs = 0;

void driveMotors(float linear, float angular) {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastDriveUs) / 1000000.0;
  lastDriveUs = nowUs;
  if (dt <= 0 || dt > 0.5) dt = 0.02;

  float maxStep = RAMP_RATE * dt;

  float vLeft  = linear - (angular * WHEEL_BASE / 2.0);
  float vRight = linear + (angular * WHEEL_BASE / 2.0);

  float targetL = constrain(vLeft  / MAX_SPEED, -1.0, 1.0);
  float targetR = constrain(vRight / MAX_SPEED, -1.0, 1.0);

  float diffL = targetL - smoothL;
  float diffR = targetR - smoothR;
  smoothL += constrain(diffL, -maxStep, maxStep);
  smoothR += constrain(diffR, -maxStep, maxStep);

  setMotor(LEFT_PWM,  LEFT_DIR,  smoothL, LEFT_PWM_BIAS);
  setMotor(RIGHT_PWM, RIGHT_DIR, smoothR, RIGHT_PWM_BIAS);
}

void stopMotors() {
  smoothL = 0.0;
  smoothR = 0.0;
  analogWrite(LEFT_PWM,  0);
  analogWrite(RIGHT_PWM, 0);
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
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(LEFT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

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

  delay(100);
  noInterrupts();
  encTicksL = 0;
  encTicksR = 0;
  prevTicksL = 0;
  prevTicksR = 0;
  interrupts();

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
    float absL = fabs(constrain(vL/MAX_SPEED,-1,1));
    float absR = fabs(constrain(vR/MAX_SPEED,-1,1));
    int pwmL = (absL < 0.01) ? 0 : constrain(PWM_MIN_MOVE + (int)(absL * (float)(255 - PWM_MIN_MOVE)) - LEFT_PWM_BIAS, PWM_MIN_MOVE, 255);
    int pwmR = (absR < 0.01) ? 0 : constrain(PWM_MIN_MOVE + (int)(absR * (float)(255 - PWM_MIN_MOVE)) - RIGHT_PWM_BIAS, PWM_MIN_MOVE, 255);
    bool timeout = (millis() - lastCmdTime > CMD_TIMEOUT_MS);
    noInterrupts();
    long tL = encTicksL;
    long tR = encTicksR;
    interrupts();
    Serial.print("D lin="); Serial.print(targetLinear, 2);
    Serial.print(" ang="); Serial.print(targetAngular, 2);
    Serial.print(" pwmL="); Serial.print(pwmL);
    Serial.print(" pwmR="); Serial.print(pwmR);
    Serial.print(" encL="); Serial.print(tL);
    Serial.print(" encR="); Serial.print(tR);
    Serial.print(" pins=");
    Serial.print(digitalRead(ENC_L_A)); Serial.print(digitalRead(ENC_L_B));
    Serial.print(digitalRead(ENC_R_A)); Serial.print(digitalRead(ENC_R_B));
    Serial.print(" tout="); Serial.println(timeout);
  }
}
