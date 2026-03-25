#include <Servo.h>

// -------------------- PINS --------------------
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 2;
const int SERVO_PIN = 5;

// -------------------- SERVO --------------------
const int SERVO_CENTER = 90;
const int SERVO_LIMIT  = 22;      // max steering authority
const int SERVO_RATE   = 10;      // how fast rudder can move (deg per loop)
const unsigned long DT_MS = 50;   // 20 Hz control loop

// -------------------- OPEN WATER TANK (4.5m wide) --------------------
// 4.5 m = 450 cm. Center is ~225 cm from each wall.
// We only want to slow/panic when actually getting close.
const int WALL_CLOSE_CM = 90;   // start slowing within 0.9m of a wall
const int WALL_PANIC_CM = 45;   // emergency steer within 0.45m of a wall

// -------------------- SPEEDS --------------------
const int PWM_FAST  = 255;      // cruising speed when centered
const int PWM_SLOW  = 180;      // near wall or off-center
const int PWM_PANIC = 95;       // emergency (gives time to rotate)

// Smooth PWM ramp (prevents sudden speed jumps)
const int PWM_RATE = 4;
int pwmNow = PWM_SLOW;

// -------------------- CONTROL GAINS --------------------
// Using your working trim-tank gains as a starting point:
float KP = 8.0;
float KD = 4.0;
float prevErr = 0.0f;

// Deadband helps remove tiny jitter near center
const float ERR_DEADBAND  = 0.04;   // smaller for open tank
const float OFFCENTER_ERR = 0.08;   // if error is large, slow down earlier

Servo rudder;
int servoPos = SERVO_CENTER;

// -------------------- TF-Luna RAW UART READER --------------------
struct Lidar {
  HardwareSerial* port;
  int dist_cm = 9999;
  unsigned long last_ms = 0;
  uint8_t state = 0;
  uint8_t buf[7];
  uint8_t idx = 0;
};

Lidar L_left, L_right;

void lidarInit(Lidar &L, HardwareSerial &S) { L.port = &S; }

void lidarPoll(Lidar &L) {
  while (L.port->available()) {
    uint8_t b = (uint8_t)L.port->read();

    if (L.state == 0) {
      if (b == 0x59) L.state = 1;
    } else if (L.state == 1) {
      if (b == 0x59) { L.state = 2; L.idx = 0; }
      else L.state = 0;
    } else {
      L.buf[L.idx++] = b;
      if (L.idx >= 7) {
        int dist = (int)L.buf[0] + ((int)L.buf[1] << 8);
        if (dist > 0 && dist < 1200) {
          L.dist_cm = dist;
          L.last_ms = millis();
        }
        L.state = 0;
      }
    }
  }
}

bool fresh(const Lidar &L, unsigned long now, unsigned long maxAgeMs = 800) {
  return (now - L.last_ms) <= maxAgeMs;
}

// -------------------- MOTOR / SERVO HELPERS --------------------
void setMotorForward(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(ENA, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void setMotorStop() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void servoToward(int target) {
  target = constrain(target, SERVO_CENTER - SERVO_LIMIT, SERVO_CENTER + SERVO_LIMIT);

  if (target > servoPos) servoPos += SERVO_RATE;
  else if (target < servoPos) servoPos -= SERVO_RATE;

  servoPos = constrain(servoPos, SERVO_CENTER - SERVO_LIMIT, SERVO_CENTER + SERVO_LIMIT);
  rudder.write(servoPos);
}

void pwmToward(int targetPwm) {
  targetPwm = constrain(targetPwm, 0, 255);
  if (targetPwm > pwmNow) pwmNow = min(pwmNow + PWM_RATE, targetPwm);
  else if (targetPwm < pwmNow) pwmNow = max(pwmNow - PWM_RATE, targetPwm);
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  rudder.attach(SERVO_PIN);
  rudder.write(SERVO_CENTER);

  Serial.begin(115200);

  Serial2.begin(115200); // Left sensor (RX2=17 TX2=16)
  Serial3.begin(115200); // Right sensor (RX3=15 TX3=14)
  lidarInit(L_left, Serial2);
  lidarInit(L_right, Serial3);

  Serial.println("Open-water wall-centering (PD + SLOW + PANIC).");
  Serial.println("Format: L | R | e | servo | pwm | mode");
}

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  if (now - lastLoop < DT_MS) return;
  lastLoop = now;

  lidarPoll(L_left);
  lidarPoll(L_right);

  bool leftFresh  = fresh(L_left, now);
  bool rightFresh = fresh(L_right, now);

  if (!leftFresh && !rightFresh) {
    servoToward(SERVO_CENTER);
    setMotorStop();
    return;
  }

  int dL = L_left.dist_cm;
  int dR = L_right.dist_cm;

  // -------------------- PANIC MODE --------------------
  // If too close to a wall, force max steering away + slow down hard.
  if (dL < WALL_PANIC_CM || dR < WALL_PANIC_CM) {
    int panicTarget;
    if (dL < dR) panicTarget = SERVO_CENTER + SERVO_LIMIT; // steer RIGHT
    else         panicTarget = SERVO_CENTER - SERVO_LIMIT; // steer LEFT

    servoToward(panicTarget);
    pwmToward(PWM_PANIC);
    setMotorForward(pwmNow);

    Serial.print(dL); Serial.print(" | ");
    Serial.print(dR); Serial.print(" | ");
    Serial.print("PANIC | ");
    Serial.print(servoPos); Serial.print(" | ");
    Serial.println(pwmNow);
    return;
  }

  // -------------------- NORMAL PD CENTERING --------------------
  float error = (float)(dR - dL) / (float)(dR + dL);
  if (fabs(error) < ERR_DEADBAND) error = 0.0f;

  float dErr = (error - prevErr) / (DT_MS / 1000.0f);
  prevErr = error;

  float control = KP * error + KD * dErr;

  int steerCmd = (int)(control * SERVO_LIMIT);
  int target = SERVO_CENTER + steerCmd;
  servoToward(target);

  // -------------------- SPEED LOGIC --------------------
  bool closeToWall = (dL < WALL_CLOSE_CM) || (dR < WALL_CLOSE_CM);
  bool offCenter   = (fabs(error) > OFFCENTER_ERR);

  int pwmCmd = (closeToWall || offCenter) ? PWM_SLOW : PWM_FAST;
  pwmToward(pwmCmd);
  setMotorForward(pwmNow);

  // -------------------- DEBUG PRINT --------------------
  const char* mode = (closeToWall || offCenter) ? "SLOW" : "FAST";

  Serial.print(dL); Serial.print(" | ");
  Serial.print(dR); Serial.print(" | ");
  Serial.print(error, 3); Serial.print(" | ");
  Serial.print(servoPos); Serial.print(" | ");
  Serial.print(pwmNow); Serial.print(" | ");
  Serial.println(mode);
}