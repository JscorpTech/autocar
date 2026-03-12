#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// --- motorlar (L298N) ---
// ikkala orqa motor bir xil tezlikda ishlaydi (Ackermann)
#define MOTOR_A_IN1  16
#define MOTOR_A_IN2  17
#define MOTOR_A_ENA  18

#define MOTOR_B_IN3  19
#define MOTOR_B_IN4  20
#define MOTOR_B_ENB  21

// --- rul servosi ---
#define SERVO_PIN     7
#define SERVO_CENTER  90     // to'g'ri holat
#define MAX_STEER     30     // maksimal burchak (bir tomonga)

// --- encoderlar ---
#define ENC_LEFT_PIN   4
#define ENC_RIGHT_PIN  5
#define PULSES_PER_REV 4
#define DEBOUNCE_MS    10

// --- I2C (kompas) ---
#define I2C_SDA  8
#define I2C_SCL  9

// --- ultrasonic ---
#define TRIG_PIN  10
#define ECHO_PIN  11

// --- Pi bilan serial ---
#define RPI_TX    43
#define RPI_RX    44
#define RPI_BAUD  115200

#define DATA_INTERVAL  100
#define CMD_TIMEOUT    2000

// encoder
volatile unsigned long encLeftCount = 0;
volatile unsigned long encRightCount = 0;
volatile unsigned long lastPulseL = 0;
volatile unsigned long lastPulseR = 0;
unsigned long prevEncL = 0;
unsigned long prevEncR = 0;

// RPM
unsigned long lastRpmTime = 0;
float rpmLeft = 0;
float rpmRight = 0;

// kompas
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
bool compassOk = false;
float heading = 0.0;

// haydash
int driveSpeed = 0;
int steerAngle = 0;     // -30..+30 daraja

// ultrasonic
float distanceM = 999.0;

// aloqa
String inputBuf = "";
unsigned long lastDataSend = 0;
unsigned long lastCmdTime = 0;


void IRAM_ATTR onEncLeft() {
  unsigned long now = millis();
  if (now - lastPulseL > DEBOUNCE_MS) {
    encLeftCount++;
    lastPulseL = now;
  }
}

void IRAM_ATTR onEncRight() {
  unsigned long now = millis();
  if (now - lastPulseR > DEBOUNCE_MS) {
    encRightCount++;
    lastPulseR = now;
  }
}


// --- servo boshqaruv ---

void servoWrite(int angleDeg) {
  // 0-180 gradus -> 500-2500 mikrosekund
  angleDeg = constrain(angleDeg, 0, 180);
  int us = map(angleDeg, 0, 180, 500, 2500);
  // 50Hz da 16-bit resolution
  int duty = (int)((float)us / 20000.0 * 65536.0);
  ledcWrite(SERVO_PIN, duty);
}

void setSteering(int angle) {
  // angle: -MAX_STEER..+MAX_STEER (manfiy=chapga, musbat=o'ngga)
  steerAngle = constrain(angle, -MAX_STEER, MAX_STEER);
  int servoAngle = SERVO_CENTER + steerAngle;
  servoWrite(servoAngle);
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(RPI_BAUD, SERIAL_8N1, RPI_RX, RPI_TX);

  // motorlar
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);

  // motor PWM - 5kHz, 8-bit
  ledcAttach(MOTOR_A_ENA, 5000, 8);
  ledcAttach(MOTOR_B_ENB, 5000, 8);

  // servo PWM - 50Hz, 16-bit
  ledcAttach(SERVO_PIN, 50, 16);
  setSteering(0);  // rul to'g'ri

  stopMotors();

  // encoderlar
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), onEncLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), onEncRight, FALLING);

  // ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // kompas
  Wire.begin(I2C_SDA, I2C_SCL);
  compassOk = compass.begin();
  Serial.println(compassOk ? "kompas ok" : "kompas topilmadi");

  Serial.println("mashina tayyor");
  lastCmdTime = millis();
  lastRpmTime = millis();
}


// --- motorlar ---
// Ackermann: ikkala motor bir xil tezlikda (differensial yo'q)

void setMotors(int spd) {
  spd = constrain(spd, -255, 255);

  if (spd > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
    ledcWrite(MOTOR_A_ENA, spd);
    ledcWrite(MOTOR_B_ENB, spd);
  } else if (spd < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
    ledcWrite(MOTOR_A_ENA, -spd);
    ledcWrite(MOTOR_B_ENB, -spd);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
    ledcWrite(MOTOR_A_ENA, 0);
    ledcWrite(MOTOR_B_ENB, 0);
  }
}

void stopMotors() {
  setMotors(0);
  setSteering(0);
  driveSpeed = 0;
  steerAngle = 0;
}


// --- sensorlar ---

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999.0;
  return (dur * 0.0343 / 2.0) / 100.0;  // metrga
}

float readCompass() {
  if (!compassOk) return -1.0;

  sensors_event_t event;
  compass.getEvent(&event);

  float h = atan2(event.magnetic.y, event.magnetic.x);
  h += 5.0 * (PI / 180.0);  // Toshkent deklinatsiyasi

  if (h < 0) h += 2 * PI;
  if (h > 2 * PI) h -= 2 * PI;

  return h * (180.0 / PI);
}

void updateRPM() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmTime;

  if (elapsed >= 200) {
    noInterrupts();
    unsigned long dL = encLeftCount - prevEncL;
    unsigned long dR = encRightCount - prevEncR;
    prevEncL = encLeftCount;
    prevEncR = encRightCount;
    interrupts();

    float sec = elapsed / 1000.0;
    rpmLeft = ((float)dL / PULSES_PER_REV / sec) * 60.0;
    rpmRight = ((float)dR / PULSES_PER_REV / sec) * 60.0;
    lastRpmTime = now;
  }
}


// --- Pi bilan aloqa ---

void processCommand(String cmd) {
  cmd.trim();
  lastCmdTime = millis();

  if (cmd.startsWith("CMD:")) {
    // CMD:speed,steer_angle
    String params = cmd.substring(4);
    int idx = params.indexOf(',');
    if (idx > 0) {
      driveSpeed = constrain(params.substring(0, idx).toInt(), -255, 255);
      int angle = constrain(params.substring(idx + 1).toInt(), -MAX_STEER, MAX_STEER);

      setMotors(driveSpeed);
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
    encLeftCount = 0;
    encRightCount = 0;
    prevEncL = 0;
    prevEncR = 0;
    interrupts();
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
  unsigned long eL = encLeftCount;
  unsigned long eR = encRightCount;
  interrupts();

  // DATA:encL,encR,heading,rpmL,rpmR,dist(metr)
  String data = "DATA:";
  data += String(eL) + ",";
  data += String(eR) + ",";
  data += String(heading, 1) + ",";
  data += String(rpmLeft, 1) + ",";
  data += String(rpmRight, 1) + ",";
  data += String(distanceM, 2);

  Serial1.println(data);
  lastDataSend = millis();
}


void loop() {
  readCommands();

  updateRPM();
  heading = readCompass();

  static unsigned long lastUltra = 0;
  if (millis() - lastUltra >= 100) {
    distanceM = readUltrasonic();
    lastUltra = millis();

    if (distanceM < 0.5 && distanceM > 0) {
      stopMotors();
      Serial1.println("WARN:OBSTACLE");
    }
  }

  sendTelemetry();

  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (driveSpeed != 0) {
      stopMotors();
      Serial1.println("WARN:TIMEOUT");
    }
  }
}
