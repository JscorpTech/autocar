// --- BTS7960 motors ---
// front motors
#define FRONT_RPWM   18
#define FRONT_LPWM   19
// rear motors
#define REAR_RPWM    21
#define REAR_LPWM    22
// steering motor
#define STEER_RPWM   26
#define STEER_LPWM   27

// --- HC-SR04 ultrasonic sensors (6 total) ---
#define FRONT_TRIG        13
#define FRONT_ECHO        14
#define FRONT_RIGHT_TRIG  15
#define FRONT_RIGHT_ECHO  16
#define FRONT_LEFT_TRIG   17
#define FRONT_LEFT_ECHO   23
#define RIGHT_TRIG        25
#define RIGHT_ECHO        32
#define LEFT_TRIG         33
#define LEFT_ECHO         34   // input only, no internal pull-up/pull-down
#define REAR_TRIG         4
#define REAR_ECHO         35   // input only

// --- LM393 IR odometry sensors (4 total) ---
#define ENC_FRONT_RIGHT  36   // input only (VP), no internal pull-up/pull-down
#define ENC_FRONT_LEFT   39   // input only (VN), no internal pull-up/pull-down
#define ENC_REAR_RIGHT    5   // strapping pin: must be HIGH at boot
#define ENC_REAR_LEFT     2   // strapping pin: must be LOW at boot

// --- Serial connection to Raspberry Pi ---
#define RPI_TX    43
#define RPI_RX    44
#define RPI_BAUD  115200

#define PULSES_PER_REV  4
#define DEBOUNCE_MS     10
#define WHEEL_CIRC      0.785f   // PI * 0.25m
#define WHEEL_BASE      1.8f     // metres
#define MAX_STEER       30       // maximum steering angle (degrees)

#define DATA_INTERVAL   100
#define CMD_TIMEOUT     2000
#define OBSTACLE_DIST_M 0.5f
#define ULTRA_TIMEOUT   20000    // 20ms -> ~3.4m max range

// --- encoder variables ---
volatile unsigned long encFrontRight = 0;
volatile unsigned long encFrontLeft  = 0;
volatile unsigned long encRearRight  = 0;
volatile unsigned long encRearLeft   = 0;
volatile unsigned long lastPulseFR   = 0;
volatile unsigned long lastPulseFL   = 0;
volatile unsigned long lastPulseRR   = 0;
volatile unsigned long lastPulseRL   = 0;
unsigned long prevEncFR = 0, prevEncFL = 0;
unsigned long prevEncRR = 0, prevEncRL = 0;

// --- RPM and heading ---
unsigned long lastRpmTime = 0;
float rpmLeft  = 0.0;
float rpmRight = 0.0;
float heading  = 0.0;   // odometry-based, degrees

// --- distances ---
float distFront      = 999.0;
float distFrontRight = 999.0;
float distFrontLeft  = 999.0;
float distRight      = 999.0;
float distLeft       = 999.0;
float distRear       = 999.0;

// --- haydash holati ---
int driveSpeed = 0;
int steerAngle = 0;   // -MAX_STEER..+MAX_STEER degrees

// --- aloqa ---
String inputBuf = "";
unsigned long lastDataSend = 0;
unsigned long lastCmdTime  = 0;


// =====================================================================
// ISR - encoder pulses
// =====================================================================

void IRAM_ATTR isr_FrontRight() {
  unsigned long now = millis();
  if (now - lastPulseFR > DEBOUNCE_MS) { encFrontRight++; lastPulseFR = now; }
}
void IRAM_ATTR isr_FrontLeft() {
  unsigned long now = millis();
  if (now - lastPulseFL > DEBOUNCE_MS) { encFrontLeft++;  lastPulseFL = now; }
}
void IRAM_ATTR isr_RearRight() {
  unsigned long now = millis();
  if (now - lastPulseRR > DEBOUNCE_MS) { encRearRight++;  lastPulseRR = now; }
}
void IRAM_ATTR isr_RearLeft() {
  unsigned long now = millis();
  if (now - lastPulseRL > DEBOUNCE_MS) { encRearLeft++;   lastPulseRL = now; }
}


// =====================================================================
// BTS7960 motor control
// =====================================================================

void setBTS7960(int rpwmPin, int lpwmPin, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    ledcWrite(rpwmPin, spd);
    ledcWrite(lpwmPin, 0);
  } else if (spd < 0) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, -spd);
  } else {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, 0);
  }
}

void setMotors(int spd) {
  // front and rear motors at the same speed (Ackermann)
  spd = constrain(spd, -255, 255);
  driveSpeed = spd;
  setBTS7960(FRONT_RPWM, FRONT_LPWM, spd);
  setBTS7960(REAR_RPWM,  REAR_LPWM,  spd);
}

void setSteering(int angleDeg) {
  // angleDeg: -MAX_STEER..+MAX_STEER (negative=left, positive=right)
  // Proportional PWM: -30 -> -255, 0 -> 0, +30 -> +255
  angleDeg = constrain(angleDeg, -MAX_STEER, MAX_STEER);
  steerAngle = angleDeg;
  int pwm = (int)((float)angleDeg / (float)MAX_STEER * 255.0f);
  setBTS7960(STEER_RPWM, STEER_LPWM, pwm);
}

void stopMotors() {
  setMotors(0);
  setSteering(0);
  driveSpeed = 0;
  steerAngle = 0;
}


// =====================================================================
// Ultrasonic sensor reading
// =====================================================================

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT);
  if (dur == 0) return 999.0f;
  return (dur * 0.0343f / 2.0f) / 100.0f;   // convert to metres
}

// Read sensors one at a time (round-robin to reduce blocking)
void updateNextSensor() {
  static uint8_t idx = 0;
  switch (idx) {
    case 0: distFront      = readUltrasonic(FRONT_TRIG,       FRONT_ECHO);       break;
    case 1: distFrontRight = readUltrasonic(FRONT_RIGHT_TRIG, FRONT_RIGHT_ECHO); break;
    case 2: distFrontLeft  = readUltrasonic(FRONT_LEFT_TRIG,  FRONT_LEFT_ECHO);  break;
    case 3: distRight      = readUltrasonic(RIGHT_TRIG,       RIGHT_ECHO);       break;
    case 4: distLeft       = readUltrasonic(LEFT_TRIG,        LEFT_ECHO);        break;
    case 5: distRear       = readUltrasonic(REAR_TRIG,        REAR_ECHO);        break;
  }
  idx = (idx + 1) % 6;
}


// =====================================================================
// Odometry: RPM and heading calculation
// =====================================================================

void updateOdometry() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmTime;
  if (elapsed < 200) return;

  noInterrupts();
  unsigned long fR = encFrontRight, fL = encFrontLeft;
  unsigned long rR = encRearRight,  rL = encRearLeft;
  interrupts();

  unsigned long dFR = fR - prevEncFR;
  unsigned long dFL = fL - prevEncFL;
  unsigned long dRR = rR - prevEncRR;
  unsigned long dRL = rL - prevEncRL;
  prevEncFR = fR; prevEncFL = fL;
  prevEncRR = rR; prevEncRL = rL;

  float sec  = elapsed / 1000.0f;
  float base = 60.0f / (float)PULSES_PER_REV / sec;
  rpmLeft  = ((dFL + dRL) / 2.0f) * base;
  rpmRight = ((dFR + dRR) / 2.0f) * base;

  // Differential odometry heading update.
  // Encoder pulses always count positively, so reverse-travel
  // distance sign is applied manually when driving backwards.
  float mpp    = WHEEL_CIRC / (float)PULSES_PER_REV;
  float dLeft  = ((dFL + dRL) / 2.0f) * mpp;
  float dRight = ((dFR + dRR) / 2.0f) * mpp;
  if (driveSpeed < 0) { dLeft = -dLeft; dRight = -dRight; }
  float dTheta = (dRight - dLeft) / WHEEL_BASE * (180.0f / PI);
  heading = fmod(heading + dTheta + 360.0f, 360.0f);

  lastRpmTime = now;
}


// =====================================================================
// Communication with Pi
// =====================================================================

void processCommand(String cmd) {
  cmd.trim();
  lastCmdTime = millis();

  if (cmd.startsWith("CMD:")) {
    // CMD:speed,steer_angle
    String params = cmd.substring(4);
    int idx = params.indexOf(',');
    if (idx > 0) {
      int spd   = constrain(params.substring(0, idx).toInt(), -255, 255);
      int angle = constrain(params.substring(idx + 1).toInt(), -MAX_STEER, MAX_STEER);
      setMotors(spd);
      setSteering(angle);
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
  }
  else if (cmd == "PING") {
    Serial1.println("PONG");
  }
  else if (cmd == "RESET_ENC") {
    noInterrupts();
    encFrontRight = encFrontLeft = encRearRight = encRearLeft = 0;
    prevEncFR     = prevEncFL   = prevEncRR    = prevEncRL   = 0;
    interrupts();
    heading = 0.0f;
    Serial1.println("ACK:RESET_ENC");
  }
}

void readCommands() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      processCommand(inputBuf);
      inputBuf = "";
    } else {
      inputBuf += c;
    }
  }
}

void sendTelemetry() {
  if (millis() - lastDataSend < DATA_INTERVAL) return;

  noInterrupts();
  unsigned long fR = encFrontRight, fL = encFrontLeft;
  unsigned long rR = encRearRight,  rL = encRearLeft;
  interrupts();

  // encL = average of left encoders, encR = average of right encoders
  // +1 for rounding before integer truncation (averaged on Raspberry Pi side)
  unsigned long encL = (fL + rL + 1) / 2;
  unsigned long encR = (fR + rR + 1) / 2;

  // DATA:encL,encR,heading,rpmL,rpmR,dFront,dFrontRight,dFrontLeft,dRight,dLeft,dRear
  String data = "DATA:";
  data += String(encL)                  + ",";
  data += String(encR)                  + ",";
  data += String(heading, 1)            + ",";
  data += String(rpmLeft,  1)           + ",";
  data += String(rpmRight, 1)           + ",";
  data += String(distFront,      2)     + ",";
  data += String(distFrontRight, 2)     + ",";
  data += String(distFrontLeft,  2)     + ",";
  data += String(distRight,      2)     + ",";
  data += String(distLeft,       2)     + ",";
  data += String(distRear,       2);

  Serial1.println(data);
  lastDataSend = millis();
}


// =====================================================================
// Setup
// =====================================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(RPI_BAUD, SERIAL_8N1, RPI_RX, RPI_TX);

  // BTS7960 drive motors PWM - 1kHz, 8-bit
  ledcAttach(FRONT_RPWM, 1000, 8);
  ledcAttach(FRONT_LPWM, 1000, 8);
  ledcAttach(REAR_RPWM,  1000, 8);
  ledcAttach(REAR_LPWM,  1000, 8);
  // BTS7960 steering motor PWM - 1kHz, 8-bit
  ledcAttach(STEER_RPWM, 1000, 8);
  ledcAttach(STEER_LPWM, 1000, 8);
  stopMotors();

  // Ultrasonic trigger/echo pins
  int trigPins[] = { FRONT_TRIG, FRONT_RIGHT_TRIG, FRONT_LEFT_TRIG,
                     RIGHT_TRIG, LEFT_TRIG, REAR_TRIG };
  int echoPins[] = { FRONT_ECHO, FRONT_RIGHT_ECHO, FRONT_LEFT_ECHO,
                     RIGHT_ECHO, LEFT_ECHO, REAR_ECHO };
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }

  // LM393 encoder pins
  // GPIO36, 39 - input only, no internal pull-up
  pinMode(ENC_FRONT_RIGHT, INPUT);
  pinMode(ENC_FRONT_LEFT,  INPUT);
  // GPIO5, 2 - strapping pins, handle with care
  pinMode(ENC_REAR_RIGHT,  INPUT_PULLUP);
  pinMode(ENC_REAR_LEFT,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_RIGHT), isr_FrontRight, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_LEFT),  isr_FrontLeft,  FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_RIGHT),  isr_RearRight,  FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_LEFT),   isr_RearLeft,   FALLING);

  lastCmdTime = millis();
  lastRpmTime = millis();
  Serial.println("car ready (BTS7960 + 6x HC-SR04 + 4x LM393)");
}


// =====================================================================
// Main loop
// =====================================================================

void loop() {
  readCommands();
  updateOdometry();

  // Read sensors one at a time (one per 50ms)
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime >= 50) {
    updateNextSensor();
    lastSensorTime = millis();
  }

  // Front obstacle check (while driving forward)
  if (driveSpeed > 0 && distFront > 0.0f && distFront < OBSTACLE_DIST_M) {
    stopMotors();
    Serial1.println("WARN:OBSTACLE");
  }
  // Rear obstacle check (while driving backward)
  if (driveSpeed < 0 && distRear > 0.0f && distRear < OBSTACLE_DIST_M) {
    stopMotors();
    Serial1.println("WARN:OBSTACLE_REAR");
  }

  sendTelemetry();

  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (driveSpeed != 0) {
      stopMotors();
      Serial1.println("WARN:TIMEOUT");
    }
  }
}
