// =====================================================================
// Autonomous Car - ESP32 Firmware
// BTS7960 motors + 6x HC-SR04 + 4x LM393, odometry heading
//
// Key parameters:
//  - WHEEL_CIRC: 1.0996m (diameter 35cm)
//  - PULSES_PER_REV: 18
//  - DEBOUNCE_MS: 5ms
//  - Odometry: 50ms update
//  - RESET_ENC: heading NOT reset (critical fix)
//  - 3 front sensors checked for obstacle
//  - Buffer overflow protection (MAX_CMD_LEN = 64)
// =====================================================================

// DEBUG mode: false = clean serial (only DATA/WARN/ACK for Raspberry Pi)
// Set true only when debugging with Arduino IDE Serial Monitor (Pi disconnected)
#define DEBUG true
#define DPRINT(x)   if(DEBUG) { Serial.print(millis()); Serial.print("ms | "); Serial.print(x); }
#define DPRINTLN(x) if(DEBUG) { Serial.print(millis()); Serial.print("ms | "); Serial.println(x); }

// HMC5883L compass removed — using odometry heading only

// --- BTS7960 motors ---
// front motors
#define FRONT_RPWM      18
#define FRONT_LPWM      19
// rear motors
#define REAR_RPWM       21
#define REAR_LPWM       22
// steering motor
#define STEER_RPWM      26
#define STEER_LPWM      27

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
// Uses USB Serial (UART0) — connect Pi via USB cable (/dev/ttyUSB0 or /dev/ttyACM0)
#define RPI_BAUD  115200

// --- PWM channels for BTS7960 (ESP32 Arduino core 2.x API) ---
#define CH_FRONT_RPWM  0
#define CH_FRONT_LPWM  1
#define CH_REAR_RPWM   2
#define CH_REAR_LPWM   3
#define CH_STEER_RPWM  4
#define CH_STEER_LPWM  5

// --- Car parameters ---
#define PULSES_PER_REV  18       // Encoder: 18 pulses/revolution (measured)
#define DEBOUNCE_MS     5        // 5ms: at 1m/s ~61ms interval, safe
#define WHEEL_CIRC      1.0996f  // PI*0.35m (diameter 35cm)
#define WHEEL_BASE      0.95f    // front-rear axle distance, metres
#define MAX_STEER       20       // maximum steering angle, degrees

// --- Communication and safety ---
#define DATA_INTERVAL   100      // telemetry send interval, ms
#define CMD_TIMEOUT     2000     // stop if no command received, ms
#define OBSTACLE_DIST_M 0.5f     // obstacle detection distance, metres
#define ULTRA_TIMEOUT   20000    // pulseIn timeout, us (~3.4m)
#define MAX_CMD_LEN     64       // buffer overflow protection


// =====================================================================
// Encoder variables
// =====================================================================
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

// RPM and heading
unsigned long lastRpmTime = 0;
float rpmLeft  = 0.0;
float rpmRight = 0.0;
float heading  = 0.0;   // current heading, degrees [0..360)

// Distances
float distFront      = 999.0;
float distFrontRight = 999.0;
float distFrontLeft  = 999.0;
float distRight      = 999.0;
float distLeft       = 999.0;
float distRear       = 999.0;

// Drive state
int driveSpeed = 0;
int steerAngle = 0;   // -MAX_STEER..+MAX_STEER degrees

// Communication
String inputBuf = "";
unsigned long lastDataSend = 0;
unsigned long lastCmdTime  = 0;


// =====================================================================
// ISR - encoder pulses (FIXED: DEBOUNCE_MS = 2)
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

void setBTS7960(int rpwmCh, int lpwmCh, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    ledcWrite(rpwmCh, spd);
    ledcWrite(lpwmCh, 0);
  } else if (spd < 0) {
    ledcWrite(rpwmCh, 0);
    ledcWrite(lpwmCh, -spd);
  } else {
    ledcWrite(rpwmCh, 0);
    ledcWrite(lpwmCh, 0);
  }
}

void setMotors(int spd) {
  // front and rear motors at the same speed (Ackermann)
  spd = constrain(spd, -255, 255);
  driveSpeed = spd;
  setBTS7960(CH_FRONT_RPWM, CH_FRONT_LPWM, spd);
  setBTS7960(CH_REAR_RPWM,  CH_REAR_LPWM,  spd);
  DPRINT("MOTOR spd="); DPRINTLN(spd);
}

void setSteering(int angleDeg) {
  // angleDeg: -MAX_STEER..+MAX_STEER (negative=left, positive=right)
  // Proportional PWM: -20 -> -255, 0 -> 0, +20 -> +255
  angleDeg = constrain(angleDeg, -MAX_STEER, MAX_STEER);
  steerAngle = angleDeg;
  int pwm = (int)((float)angleDeg / (float)MAX_STEER * 255.0f);
  setBTS7960(CH_STEER_RPWM, CH_STEER_LPWM, pwm);
  DPRINT("STEER angle="); DPRINT(angleDeg); DPRINT(" pwm="); DPRINTLN(pwm);
}

void stopMotors() {
  setMotors(0);
  setSteering(0);
  driveSpeed = 0;
  steerAngle = 0;
  DPRINTLN("STOP");
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
  return (dur * 0.0343f / 2.0f) / 100.0f;   // cm -> metres
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
// Compass has priority. If not connected, use odometry.
// FIXED: 200ms -> 50ms update interval
// =====================================================================

void updateOdometry() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmTime;
  if (elapsed < 50) return;

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

  // Odometry heading
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

  DPRINT("CMD_IN: ["); DPRINT(cmd); DPRINTLN("]");

  if (cmd.startsWith("CMD:")) {
    String params = cmd.substring(4);
    int idx = params.indexOf(',');
    if (idx > 0) {
      int spd   = constrain(params.substring(0, idx).toInt(), -255, 255);
      int angle = constrain(params.substring(idx + 1).toInt(), -MAX_STEER, MAX_STEER);
      setMotors(spd);
      setSteering(angle);
    } else {
      DPRINTLN("CMD error: comma not found");
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
    DPRINTLN("PING -> PONG sent");
  }
  else if (cmd == "RESET_ENC") {
    noInterrupts();
    encFrontRight = encFrontLeft = encRearRight = encRearLeft = 0;
    prevEncFR     = prevEncFL   = prevEncRR    = prevEncRL   = 0;
    interrupts();
    // CRITICAL FIX: heading is NOT reset here — resetting it caused
    // the car to steer hard throughout entire drive after encoder reset
    Serial.println("ACK:RESET_ENC");
    DPRINT("RESET_ENC ok, heading="); DPRINTLN(heading);
  }
  else {
    DPRINT("Unknown command: "); DPRINTLN(cmd);
  }
}

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) {
        processCommand(inputBuf);
        inputBuf = "";
      }
    } else {
      // Buffer overflow protection
      if (inputBuf.length() < MAX_CMD_LEN) {
        inputBuf += c;
      } else {
        Serial.println("[WARN] CMD buffer full, clearing");
        inputBuf = "";
      }
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
  // +1 for rounding before integer truncation
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

  Serial.println(data);
  lastDataSend = millis();
}


// =====================================================================
// Setup
// =====================================================================

void setup() {
  Serial.begin(RPI_BAUD);  // USB Serial — both debug and Raspberry Pi communication

  // BTS7960 drive motors PWM - 1kHz, 8-bit (ESP32 core 2.x API)
  ledcSetup(CH_FRONT_RPWM, 1000, 8); ledcAttachPin(FRONT_RPWM, CH_FRONT_RPWM);
  ledcSetup(CH_FRONT_LPWM, 1000, 8); ledcAttachPin(FRONT_LPWM, CH_FRONT_LPWM);
  ledcSetup(CH_REAR_RPWM,  1000, 8); ledcAttachPin(REAR_RPWM,  CH_REAR_RPWM);
  ledcSetup(CH_REAR_LPWM,  1000, 8); ledcAttachPin(REAR_LPWM,  CH_REAR_LPWM);
  // BTS7960 steering motor PWM - 1kHz, 8-bit
  ledcSetup(CH_STEER_RPWM, 1000, 8); ledcAttachPin(STEER_RPWM, CH_STEER_RPWM);
  ledcSetup(CH_STEER_LPWM, 1000, 8); ledcAttachPin(STEER_LPWM, CH_STEER_LPWM);
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

  Serial.println("=================================");
  Serial.println("Car ready (BTS7960 + 6x HC-SR04 + 4x LM393)");
  Serial.println("Heading source: Odometry");
  Serial.println("WHEEL_CIRC=1.0996m PULSES=18 DEBOUNCE=5ms ODO=50ms");
  Serial.println("DEBUG=" + String(DEBUG ? "ON" : "OFF"));
  Serial.println("Waiting for Raspberry Pi commands...");
  Serial.println("=================================");
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

  // DEBUG: print status every 2 seconds
  static unsigned long lastDebugPrint = 0;
  if (DEBUG && millis() - lastDebugPrint >= 2000) {
    noInterrupts();
    unsigned long fR = encFrontRight, fL = encFrontLeft;
    unsigned long rR = encRearRight,  rL = encRearLeft;
    interrupts();
    Serial.print("--- STATUS: spd="); Serial.print(driveSpeed);
    Serial.print(" steer=");          Serial.print(steerAngle);
    Serial.print(" head=");           Serial.print(heading, 1);
    Serial.print(" encL=");           Serial.print((fL+rL)/2);
    Serial.print(" encR=");           Serial.print((fR+rR)/2);
    Serial.print(" dFront=");         Serial.print(distFront, 2);
    Serial.print(" dFR=");            Serial.print(distFrontRight, 2);
    Serial.print(" dFL=");            Serial.print(distFrontLeft, 2);
    Serial.print(" dRear=");          Serial.println(distRear, 2);
    lastDebugPrint = millis();
  }

  // Front obstacle check (all 3 front sensors while driving forward)
  if (driveSpeed > 0) {
    float frontMin = min(distFront, min(distFrontRight, distFrontLeft));
    if (frontMin > 0.0f && frontMin < OBSTACLE_DIST_M) {
      DPRINT("OBSTACLE! frontMin="); DPRINTLN(frontMin);
      stopMotors();
      Serial.println("WARN:OBSTACLE");
    }
  }

  // Rear obstacle check (while driving backward)
  if (driveSpeed < 0 && distRear > 0.0f && distRear < OBSTACLE_DIST_M) {
    DPRINT("REAR OBSTACLE! dist="); DPRINTLN(distRear);
    stopMotors();
    Serial.println("WARN:OBSTACLE_REAR");
  }

  sendTelemetry();

  // Safety stop if no command received
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (driveSpeed != 0) {
      DPRINTLN("TIMEOUT: no command received, stopping");
      stopMotors();
      Serial.println("WARN:TIMEOUT");
    }
  }
}
