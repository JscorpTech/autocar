// --- BTS7960 motorlar ---
// oldJuft: old motorlar jufti
#define OLD_JUFT_RPWM   18
#define OLD_JUFT_LPWM   19
// orqaJuft: orqa motorlar jufti
#define ORQA_JUFT_RPWM  21
#define ORQA_JUFT_LPWM  22
// rul: rul motori
#define RUL_RPWM        26
#define RUL_LPWM        27

// --- HC-SR04 ultrasonik sensorlar (6 ta) ---
#define OLDI_TRIG        13
#define OLDI_ECHO        14
#define OLD_ONG_TRIG     15
#define OLD_ONG_ECHO     16
#define OLD_CHAP_TRIG    17
#define OLD_CHAP_ECHO    23
#define ONG_TOMON_TRIG   25
#define ONG_TOMON_ECHO   32
#define CHAP_TOMON_TRIG  33
#define CHAP_TOMON_ECHO  34   // faqat kirish, ichki pull-up/pull-down yo'q
#define ORQA_TRIG        4
#define ORQA_ECHO        35   // faqat kirish

// --- LM393 IR odometriya sensorlar (4 ta) ---
#define ENC_OLDI_ONG   36   // oldiOng - faqat kirish (VP), ichki pull-up/pull-down yo'q
#define ENC_OLDI_CHAP  39   // oldiChap - faqat kirish (VN), ichki pull-up/pull-down yo'q
#define ENC_ORQA_ONG    5   // orqaOng  - strapping pin: yuklashda HIGH bo'lishi kerak
#define ENC_ORQA_CHAP   2   // orqaChap - strapping pin: yuklashda LOW bo'lishi kerak

// --- Pi bilan serial ---
#define RPI_TX    43
#define RPI_RX    44
#define RPI_BAUD  115200

#define PULSES_PER_REV  4
#define DEBOUNCE_MS     10
#define WHEEL_CIRC      0.785f   // PI * 0.25m
#define WHEEL_BASE      1.8f     // metr
#define MAX_STEER       30       // maksimal rul burchagi (daraja)

#define DATA_INTERVAL   100
#define CMD_TIMEOUT     2000
#define OBSTACLE_DIST_M 0.5f
#define ULTRA_TIMEOUT   20000    // 20ms -> ~3.4m maksimal masofa

// --- encoder o'zgaruvchilari ---
volatile unsigned long encOldiOng  = 0;
volatile unsigned long encOldiChap = 0;
volatile unsigned long encOrqaOng  = 0;
volatile unsigned long encOrqaChap = 0;
volatile unsigned long lastPulseOO = 0;
volatile unsigned long lastPulseOC = 0;
volatile unsigned long lastPulseRO = 0;
volatile unsigned long lastPulseRC = 0;
unsigned long prevEncOO = 0, prevEncOC = 0;
unsigned long prevEncRO = 0, prevEncRC = 0;

// --- RPM va yo'nalish ---
unsigned long lastRpmTime = 0;
float rpmLeft  = 0.0;
float rpmRight = 0.0;
float heading  = 0.0;   // odometriya asosida, daraja

// --- masofalar ---
float distOldi     = 999.0;
float distOldOng   = 999.0;
float distOldChap  = 999.0;
float distOngTomon  = 999.0;
float distChapTomon = 999.0;
float distOrqa     = 999.0;

// --- haydash holati ---
int driveSpeed = 0;
int steerAngle = 0;   // -MAX_STEER..+MAX_STEER daraja

// --- aloqa ---
String inputBuf = "";
unsigned long lastDataSend = 0;
unsigned long lastCmdTime  = 0;


// =====================================================================
// ISR - encoder impulslari
// =====================================================================

void IRAM_ATTR isr_OldiOng() {
  unsigned long now = millis();
  if (now - lastPulseOO > DEBOUNCE_MS) { encOldiOng++;  lastPulseOO = now; }
}
void IRAM_ATTR isr_OldiChap() {
  unsigned long now = millis();
  if (now - lastPulseOC > DEBOUNCE_MS) { encOldiChap++; lastPulseOC = now; }
}
void IRAM_ATTR isr_OrqaOng() {
  unsigned long now = millis();
  if (now - lastPulseRO > DEBOUNCE_MS) { encOrqaOng++;  lastPulseRO = now; }
}
void IRAM_ATTR isr_OrqaChap() {
  unsigned long now = millis();
  if (now - lastPulseRC > DEBOUNCE_MS) { encOrqaChap++; lastPulseRC = now; }
}


// =====================================================================
// BTS7960 motor boshqaruv
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
  // oldJuft va orqaJuft bir xil tezlikda (Ackermann)
  spd = constrain(spd, -255, 255);
  driveSpeed = spd;
  setBTS7960(OLD_JUFT_RPWM,  OLD_JUFT_LPWM,  spd);
  setBTS7960(ORQA_JUFT_RPWM, ORQA_JUFT_LPWM, spd);
}

void setSteering(int angleDeg) {
  // angleDeg: -MAX_STEER..+MAX_STEER (manfiy=chapga, musbat=o'ngga)
  // Proportsional PWM: -30 -> -255, 0 -> 0, +30 -> +255
  angleDeg = constrain(angleDeg, -MAX_STEER, MAX_STEER);
  steerAngle = angleDeg;
  int pwm = (int)((float)angleDeg / (float)MAX_STEER * 255.0f);
  setBTS7960(RUL_RPWM, RUL_LPWM, pwm);
}

void stopMotors() {
  setMotors(0);
  setSteering(0);
  driveSpeed = 0;
  steerAngle = 0;
}


// =====================================================================
// Ultrasonik sensor o'lchash
// =====================================================================

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT);
  if (dur == 0) return 999.0f;
  return (dur * 0.0343f / 2.0f) / 100.0f;   // metrga
}

// Sensorlarni navbatma-navbat o'qish (bloklanishni kamaytirish uchun)
void updateNextSensor() {
  static uint8_t idx = 0;
  switch (idx) {
    case 0: distOldi      = readUltrasonic(OLDI_TRIG,       OLDI_ECHO);       break;
    case 1: distOldOng    = readUltrasonic(OLD_ONG_TRIG,    OLD_ONG_ECHO);    break;
    case 2: distOldChap   = readUltrasonic(OLD_CHAP_TRIG,   OLD_CHAP_ECHO);   break;
    case 3: distOngTomon  = readUltrasonic(ONG_TOMON_TRIG,  ONG_TOMON_ECHO);  break;
    case 4: distChapTomon = readUltrasonic(CHAP_TOMON_TRIG, CHAP_TOMON_ECHO); break;
    case 5: distOrqa      = readUltrasonic(ORQA_TRIG,       ORQA_ECHO);       break;
  }
  idx = (idx + 1) % 6;
}


// =====================================================================
// Odometriya: RPM va heading hisoblash
// =====================================================================

void updateOdometry() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastRpmTime;
  if (elapsed < 200) return;

  noInterrupts();
  unsigned long oO = encOldiOng,  oC = encOldiChap;
  unsigned long rO = encOrqaOng,  rC = encOrqaChap;
  interrupts();

  unsigned long dOO = oO - prevEncOO;
  unsigned long dOC = oC - prevEncOC;
  unsigned long dRO = rO - prevEncRO;
  unsigned long dRC = rC - prevEncRC;
  prevEncOO = oO; prevEncOC = oC;
  prevEncRO = rO; prevEncRC = rC;

  float sec  = elapsed / 1000.0f;
  float base = 60.0f / (float)PULSES_PER_REV / sec;
  rpmLeft  = ((dOC + dRC) / 2.0f) * base;
  rpmRight = ((dOO + dRO) / 2.0f) * base;

  // Differensial odometriya orqali heading yangilash
  // Encoder pulslari har doim musbat sanaganligi uchun
  // orqaga yurishda virtual masofa ishorasi qo'lda teskari qilinadi
  float mpp    = WHEEL_CIRC / (float)PULSES_PER_REV;
  float dLeft  = ((dOC + dRC) / 2.0f) * mpp;
  float dRight = ((dOO + dRO) / 2.0f) * mpp;
  if (driveSpeed < 0) { dLeft = -dLeft; dRight = -dRight; }
  float dTheta = (dRight - dLeft) / WHEEL_BASE * (180.0f / PI);
  heading = fmod(heading + dTheta + 360.0f, 360.0f);

  lastRpmTime = now;
}


// =====================================================================
// Pi bilan aloqa
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
    encOldiOng = encOldiChap = encOrqaOng = encOrqaChap = 0;
    prevEncOO  = prevEncOC  = prevEncRO  = prevEncRC  = 0;
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
  unsigned long oO = encOldiOng, oC = encOldiChap;
  unsigned long rO = encOrqaOng, rC = encOrqaChap;
  interrupts();

  // encL = chap encoderlar o'rtacha, encR = ong encoderlar o'rtacha
  // Floating-point bo'linish keyin butun songa o'tkaziladi (Raspberry Pi tomonida o'z navbatida o'rtacha)
  unsigned long encL = (oC + rC + 1) / 2;   // yaxlitlash uchun +1
  unsigned long encR = (oO + rO + 1) / 2;

  // DATA:encL,encR,heading,rpmL,rpmR,dOldi,dOldOng,dOldChap,dOng,dChap,dOrqa
  String data = "DATA:";
  data += String(encL)               + ",";
  data += String(encR)               + ",";
  data += String(heading, 1)         + ",";
  data += String(rpmLeft,  1)        + ",";
  data += String(rpmRight, 1)        + ",";
  data += String(distOldi,     2)    + ",";
  data += String(distOldOng,   2)    + ",";
  data += String(distOldChap,  2)    + ",";
  data += String(distOngTomon,  2)   + ",";
  data += String(distChapTomon, 2)   + ",";
  data += String(distOrqa,     2);

  Serial1.println(data);
  lastDataSend = millis();
}


// =====================================================================
// Setup
// =====================================================================

void setup() {
  Serial.begin(115200);
  Serial1.begin(RPI_BAUD, SERIAL_8N1, RPI_RX, RPI_TX);

  // BTS7960 drive motorlar PWM - 1kHz, 8-bit
  ledcAttach(OLD_JUFT_RPWM,  1000, 8);
  ledcAttach(OLD_JUFT_LPWM,  1000, 8);
  ledcAttach(ORQA_JUFT_RPWM, 1000, 8);
  ledcAttach(ORQA_JUFT_LPWM, 1000, 8);
  // BTS7960 rul motori PWM - 1kHz, 8-bit
  ledcAttach(RUL_RPWM, 1000, 8);
  ledcAttach(RUL_LPWM, 1000, 8);
  stopMotors();

  // Ultrasonik trigger/echo pinlar
  int trigPins[] = { OLDI_TRIG, OLD_ONG_TRIG, OLD_CHAP_TRIG,
                     ONG_TOMON_TRIG, CHAP_TOMON_TRIG, ORQA_TRIG };
  int echoPins[] = { OLDI_ECHO, OLD_ONG_ECHO, OLD_CHAP_ECHO,
                     ONG_TOMON_ECHO, CHAP_TOMON_ECHO, ORQA_ECHO };
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }

  // LM393 encoder pinlar
  // GPIO36, 39 - faqat kirish, ichki pull-up yo'q
  pinMode(ENC_OLDI_ONG,  INPUT);
  pinMode(ENC_OLDI_CHAP, INPUT);
  // GPIO5, 2 - strapping pinlar, ehtiyotkorlik bilan
  pinMode(ENC_ORQA_ONG,  INPUT_PULLUP);
  pinMode(ENC_ORQA_CHAP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_OLDI_ONG),  isr_OldiOng,  FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_OLDI_CHAP), isr_OldiChap, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_ORQA_ONG),  isr_OrqaOng,  FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_ORQA_CHAP), isr_OrqaChap, FALLING);

  lastCmdTime = millis();
  lastRpmTime = millis();
  Serial.println("mashina tayyor (BTS7960 + 6x HC-SR04 + 4x LM393)");
}


// =====================================================================
// Asosiy tsikl
// =====================================================================

void loop() {
  readCommands();
  updateOdometry();

  // Sensorlarni navbatma-navbat o'qish (har 50ms da bitta)
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime >= 50) {
    updateNextSensor();
    lastSensorTime = millis();
  }

  // Old to'siq tekshiruvi (oldinga yurayotganda)
  if (driveSpeed > 0 && distOldi > 0.0f && distOldi < OBSTACLE_DIST_M) {
    stopMotors();
    Serial1.println("WARN:OBSTACLE");
  }
  // Orqa to'siq tekshiruvi (orqaga yurayotganda)
  if (driveSpeed < 0 && distOrqa > 0.0f && distOrqa < OBSTACLE_DIST_M) {
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
