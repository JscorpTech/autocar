/******************************************************************************
 * BARCHA SENSORLAR UCHUN UMUMLASHTIRUVCHI KOD (TUZATILGAN)
 * ESP32-S3 + 6 ta sensor bir vaqtda ishlaydi
 * 
 * Sensorlar:
 *   1. HMC5883L - Magnitometr (Kompas)
 *   2. BMP280 - Bosim va Harorat
 *   3. DHT22 - Harorat va Namlik
 *   4. MQ135 - Gaz sensori (CO2, NH3, Benzol)
 *   5. RadiationD-v1.1 - Geiger counter (Radiatsiya)
 *   6. KY-037 - Ovoz sensori
 * 
 * Yaratilgan: 2026-03-22
 * Versiya: 2.1 (Xatoliklar tuzatilgan)
 ******************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

// ==================== PIN TA'RIFLARI ====================
// I2C pinlari
#define I2C_SDA_1 4      // HMC5883L
#define I2C_SCL_1 5
#define I2C_SDA_2 8      // BMP280
#define I2C_SCL_2 9

// DHT22
#define DHTPIN 15
#define DHTTYPE DHT22

// MQ135
#define MQ135_PIN 6

// RadiationD-v1.1
#define GEIGER_PIN 17

// KY-037
#define SOUND_ANALOG_PIN 18
#define SOUND_DIGITAL_PIN 21

// ==================== SOZLAMALAR ====================
// ADC sozlamalari
#define ADC_MAX 4095.0
#define ADC_VOLTAGE 3.3

// O'qish intervallari (ms)
#define READ_INTERVAL 2000  // 2 sekund

// Geiger counter sozlamalari
#define GEIGER_SAMPLE_INTERVAL 60000  // 1 daqiqa
#define CONVERSION_FACTOR 0.00812037037037
#define TUBE_NOISE_CPM 12.0

// MQ135 sozlamalari
#define MQ135_SAMPLES 30
#define MQ135_TYPICAL_RATIO 3.6

// KY-037 sozlamalari
#define SOUND_SAMPLES 30
#define SOUND_QUIET 500
#define SOUND_LOW 1500
#define SOUND_MEDIUM 2500

// ==================== O'ZGARUVCHILAR ====================
// Vaqtni boshqarish
unsigned long lastReadTime = 0;
unsigned long lastGeigerTime = 0;

// Sensor obyektlari
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);

// Sensor ma'lumotlari
struct SensorData {
  // HMC5883L
  float magX, magY, magZ;
  float heading;
  String direction;
  bool magOk;
  String magStatus;
  
  // BMP280
  float bmpTemp;
  float pressure;
  float altitude;
  bool bmpOk;
  String bmpStatus;
  
  // DHT22
  float dhtTemp;
  float humidity;
  float heatIndex;
  bool dhtOk;
  String dhtStatus;
  
  // MQ135
  int mq135Value;
  float co2;
  float nh3;
  float benzene;
  String airQuality;
  bool mq135Ok;
  String mq135Status;
  
  // RadiationD
  unsigned long geigerCounts;
  float cpm;
  float uSvh;
  String radiationLevel;
  bool geigerOk;
  String geigerStatus;
  
  // KY-037
  int soundAnalog;
  float soundDB;
  bool soundDigital;
  String soundLevel;
  bool soundOk;
  String soundStatus;
};

SensorData data;

// Geiger counter interrupt
volatile unsigned long geigerPulseCount = 0;

// ==================== INTERRUPT FUNKSIYASI ====================
void IRAM_ATTR onGeigerPulse() {
  geigerPulseCount++;
}

// ==================== SENSORLARNI TEKSHIRISH FUNKSIYALARI ====================
bool checkHMC5883L() {
  Wire.begin(I2C_SDA_1, I2C_SCL_1);
  Wire.beginTransmission(0x1E);  // HMC5883L I2C manzili
  byte error = Wire.endTransmission();
  return (error == 0);
}

bool checkBMP280() {
  Wire.begin(I2C_SDA_2, I2C_SCL_2);
  Wire.beginTransmission(0x76);
  byte error = Wire.endTransmission();
  if (error == 0) return true;
  
  Wire.beginTransmission(0x77);
  error = Wire.endTransmission();
  return (error == 0);
}

bool checkDHT22() {
  pinMode(DHTPIN, INPUT_PULLUP);
  delay(100);
  int reading = digitalRead(DHTPIN);
  return (reading == HIGH || reading == LOW);
}

bool checkMQ135() {
  pinMode(MQ135_PIN, INPUT);
  int value = analogRead(MQ135_PIN);
  return (value > 10 && value < 4085);
}

bool checkGeiger() {
  pinMode(GEIGER_PIN, INPUT_PULLUP);
  delay(100);
  int reading = digitalRead(GEIGER_PIN);
  return (reading == HIGH || reading == LOW);
}

bool checkKY037() {
  pinMode(SOUND_ANALOG_PIN, INPUT);
  int value = analogRead(SOUND_ANALOG_PIN);
  return (value > 5);
}

// ==================== SENSORLARNI ISHGA TUSHIRISH ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║     ESP32-S3 MULTI-SENSOR MONITORING SYSTEM                               ║");
  Serial.println("║                    6-IN-1 SENSOR HUB                                      ║");
  Serial.println("║                    Ulanish Holati Tekshiruvi                               ║");
  Serial.println("╚════════════════════════════════════════════════════════════════════════════╝");
  Serial.println("\n📡 Sensorlarni tekshirish va ishga tushirish...\n");
  
  // ADC sozlamalari
  analogReadResolution(12);
  
  Serial.println("╔════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║                    SENSORLARNI ULANGANLIGINI TEKSHIRISH                    ║");
  Serial.println("╠════════════════════════════════════════════════════════════════════════════╣");
  
  // 1. HMC5883L - Magnitometr tekshirish
  Serial.print("║ 🧭 HMC5883L (Magnitometr)        : ");
  if (checkHMC5883L()) {
    if (!mag.begin()) {
      data.magOk = false;
      data.magStatus = "ULANMAGAN";
      Serial.println("❌ ULANMAGAN     ║");
    } else {
      data.magOk = true;
      data.magStatus = "ULANGAN ✅";
      Serial.println("✅ ULANGAN       ║");
    }
  } else {
    data.magOk = false;
    data.magStatus = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  // 2. BMP280 - Bosim sensori tekshirish
  Serial.print("║ 🌡  BMP280 (Bosim/Harorat)      : ");
  if (checkBMP280()) {
    if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
      data.bmpOk = false;
      data.bmpStatus = "ULANMAGAN";
      Serial.println("❌ ULANMAGAN     ║");
    } else {
      if (bmp.begin(0x76) || bmp.begin(0x77)) {
        data.bmpOk = true;
        data.bmpStatus = "ULANGAN ✅";
        Serial.println("✅ ULANGAN       ║");
        if (data.bmpOk) {
          bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X16,
                          Adafruit_BMP280::STANDBY_MS_500);
        }
      } else {
        data.bmpOk = false;
        data.bmpStatus = "ULANMAGAN";
        Serial.println("❌ ULANMAGAN     ║");
      }
    }
  } else {
    data.bmpOk = false;
    data.bmpStatus = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  // 3. DHT22 - Harorat/Namlik tekshirish
  Serial.print("║ 💧 DHT22 (Harorat/Namlik)      : ");
  if (checkDHT22()) {
    dht.begin();
    delay(500);
    float testHum = dht.readHumidity();
    if (isnan(testHum)) {
      data.dhtOk = false;
      data.dhtStatus = "ULANMAGAN";
      Serial.println("❌ ULANMAGAN     ║");
    } else {
      data.dhtOk = true;
      data.dhtStatus = "ULANGAN ✅";
      Serial.println("✅ ULANGAN       ║");
    }
  } else {
    data.dhtOk = false;
    data.dhtStatus = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  // 4. MQ135 - Gaz sensori tekshirish
  Serial.print("║ 🌫  MQ135 (Gaz sensori)         : ");
  if (checkMQ135()) {
    pinMode(MQ135_PIN, INPUT);
    Serial.println("🔧 Kalibratsiya...");
    calibrateMQ135();
    data.mq135Ok = true;
    data.mq135Status = "ULANGAN ✅";
    Serial.println("║                                   ✅ ULANGAN       ║");
  } else {
    data.mq135Ok = false;
    data.mq135Status = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  // 5. RadiationD-v1.1 - Geiger counter tekshirish
  Serial.print("║ ☢️  RadiationD-v1.1 (Geiger)    : ");
  if (checkGeiger()) {
    pinMode(GEIGER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), onGeigerPulse, FALLING);
    data.geigerOk = true;
    data.geigerStatus = "ULANGAN ✅";
    Serial.println("✅ ULANGAN       ║");
    lastGeigerTime = millis();
  } else {
    data.geigerOk = false;
    data.geigerStatus = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  // 6. KY-037 - Ovoz sensori tekshirish
  Serial.print("║ 🔊 KY-037 (Ovoz sensori)        : ");
  if (checkKY037()) {
    pinMode(SOUND_DIGITAL_PIN, INPUT);
    pinMode(SOUND_ANALOG_PIN, INPUT);
    data.soundOk = true;
    data.soundStatus = "ULANGAN ✅";
    Serial.println("✅ ULANGAN       ║");
  } else {
    data.soundOk = false;
    data.soundStatus = "ULANMAGAN";
    Serial.println("❌ ULANMAGAN     ║");
  }
  
  Serial.println("╚════════════════════════════════════════════════════════════════════════════╝");
  
  // Umumiy hisobot
  int connectedCount = 0;
  int totalSensors = 6;
  if (data.magOk) connectedCount++;
  if (data.bmpOk) connectedCount++;
  if (data.dhtOk) connectedCount++;
  if (data.mq135Ok) connectedCount++;
  if (data.geigerOk) connectedCount++;
  if (data.soundOk) connectedCount++;
  
  Serial.println("\n╔════════════════════════════════════════════════════════════════════════════╗");
  Serial.printf("║  📊 UMUMIY HOLAT: %d/%d sensor ulangan                                   ║\n", connectedCount, totalSensors);
  Serial.println("╚════════════════════════════════════════════════════════════════════════════╝");
  
  if (connectedCount == 0) {
    Serial.println("\n⚠️  HECH QANDAY SENSOR ULANGAN EMAS!");
    Serial.println("⚠️  Ulanishlarni tekshiring va qayta yuklang.\n");
  } else {
    Serial.println("\n✅ BARCHA ULANGAN SENSORLAR TAYYOR! MA'LUMOTLAR KELMOQDA\n");
  }
  
  delay(2000);
}

// ==================== MQ135 KALIBRATSIYA ====================
float mq135_RO = 10.0;

void calibrateMQ135() {
  if (!data.mq135Ok) return;
  
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(MQ135_PIN);
    delay(50);
  }
  float avgRaw = sum / 100.0;
  float voltage = avgRaw * (ADC_VOLTAGE / ADC_MAX);
  float RL = 10.0;
  float RS = RL * ((ADC_VOLTAGE - voltage) / voltage);
  mq135_RO = RS / MQ135_TYPICAL_RATIO;
}

// ==================== SENSORLARNI O'QISH ====================
void readAllSensors() {
  // 1. HMC5883L - Magnitometr (faqat ulangan bo'lsa)
  if (data.magOk) {
    sensors_event_t event;
    mag.getEvent(&event);
    data.magX = event.magnetic.x;
    data.magY = event.magnetic.y;
    data.magZ = event.magnetic.z;
    
    float heading = atan2(data.magY, data.magX);
    float declinationAngle = 0.096;
    heading += declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    data.heading = heading * 180 / PI;
    
    if (data.heading >= 337.5 || data.heading < 22.5) data.direction = "Shimol (N)";
    else if (data.heading >= 22.5 && data.heading < 67.5) data.direction = "Shimoliy-Sharq (NE)";
    else if (data.heading >= 67.5 && data.heading < 112.5) data.direction = "Sharq (E)";
    else if (data.heading >= 112.5 && data.heading < 157.5) data.direction = "Janubiy-Sharq (SE)";
    else if (data.heading >= 157.5 && data.heading < 202.5) data.direction = "Janub (S)";
    else if (data.heading >= 202.5 && data.heading < 247.5) data.direction = "Janubiy-G'arb (SW)";
    else if (data.heading >= 247.5 && data.heading < 292.5) data.direction = "G'arb (W)";
    else data.direction = "Shimoliy-G'arb (NW)";
  }
  
  // 2. BMP280 - Bosim sensori (faqat ulangan bo'lsa)
  if (data.bmpOk) {
    data.bmpTemp = bmp.readTemperature();
    data.pressure = bmp.readPressure() / 100.0F;
    data.altitude = bmp.readAltitude(1013.25);
  }
  
  // 3. DHT22 - Harorat/Namlik (faqat ulangan bo'lsa)
  if (data.dhtOk) {
    data.humidity = dht.readHumidity();
    data.dhtTemp = dht.readTemperature();
    if (!isnan(data.humidity) && !isnan(data.dhtTemp)) {
      data.heatIndex = dht.computeHeatIndex(data.dhtTemp, data.humidity, false);
    } else {
      data.dhtOk = false;
      data.dhtStatus = "ULANMAGAN";
    }
  }
  
  // 4. MQ135 - Gaz sensori (faqat ulangan bo'lsa)
  if (data.mq135Ok) {
    long sum = 0;
    for (int i = 0; i < MQ135_SAMPLES; i++) {
      sum += analogRead(MQ135_PIN);
      delay(5);
    }
    data.mq135Value = sum / MQ135_SAMPLES;
    
    float voltage = data.mq135Value * (ADC_VOLTAGE / ADC_MAX);
    float RL = 10.0;
    float RS = RL * ((ADC_VOLTAGE - voltage) / voltage);
    float ratio = RS / mq135_RO;
    
    data.co2 = 116.602 * pow(ratio, -2.769);
    data.nh3 = 58.500 * pow(ratio, -2.182);
    data.benzene = 12.500 * pow(ratio, -2.862);
    
    if (data.co2 < 450) data.airQuality = "Yaxshi";
    else if (data.co2 < 800) data.airQuality = "O'rtacha";
    else if (data.co2 < 1200) data.airQuality = "Yomon";
    else data.airQuality = "Xavfli";
  }
  
  // 5. RadiationD-v1.1 - Geiger counter (faqat ulangan bo'lsa)
  if (data.geigerOk) {
    unsigned long now = millis();
    if (now - lastGeigerTime >= GEIGER_SAMPLE_INTERVAL) {
      noInterrupts();
      data.geigerCounts = geigerPulseCount;
      geigerPulseCount = 0;
      interrupts();
      
      data.cpm = data.geigerCounts * (60000.0 / GEIGER_SAMPLE_INTERVAL);
      // TUZATILGAN: float qiymatlarni bir xil turga keltirish
      float correctedCPM = data.cpm - TUBE_NOISE_CPM;
      if (correctedCPM < 0) correctedCPM = 0;
      data.uSvh = correctedCPM * CONVERSION_FACTOR;
      
      if (data.uSvh < 0.1) data.radiationLevel = "Normal fon ✅";
      else if (data.uSvh < 0.3) data.radiationLevel = "O'rtacha ⚠️";
      else if (data.uSvh < 1.0) data.radiationLevel = "Yuqori ⚠️⚠️";
      else data.radiationLevel = "KRITIK 🚨";
      
      lastGeigerTime = now;
    }
  }
  
  // 6. KY-037 - Ovoz sensori (faqat ulangan bo'lsa)
  if (data.soundOk) {
    long sum = 0;
    for (int i = 0; i < SOUND_SAMPLES; i++) {
      sum += analogRead(SOUND_ANALOG_PIN);
      delay(2);
    }
    data.soundAnalog = sum / SOUND_SAMPLES;
    data.soundDB = (data.soundAnalog / ADC_MAX) * 100;
    data.soundDigital = digitalRead(SOUND_DIGITAL_PIN);
    
    if (data.soundAnalog < SOUND_QUIET) data.soundLevel = "Juda tinch 🔇";
    else if (data.soundAnalog < SOUND_LOW) data.soundLevel = "Past 🔈";
    else if (data.soundAnalog < SOUND_MEDIUM) data.soundLevel = "O'rtacha 🔉";
    else data.soundLevel = "Yuqori 🔊";
  }
}

// ==================== MA'LUMOTLARNI CHIQARISH ====================
void printAllData() {
  Serial.println("╔════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║                         📊 SENSOR MA'LUMOTLARI                             ║");
  Serial.println("╠════════════════════════════════════════════════════════════════════════════╣");
  
  // HMC5883L - Magnitometr
  Serial.println("║");
  Serial.printf("║ 🧭 MAGNITOMETR (HMC5883L)           Holat: %s\n", data.magStatus.c_str());
  if (data.magOk) {
    Serial.printf("║    X: %.2f uT  |  Y: %.2f uT  |  Z: %.2f uT\n", data.magX, data.magY, data.magZ);
    Serial.printf("║    Yo'nalish: %.1f°  (%s)\n", data.heading, data.direction.c_str());
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // BMP280 - Bosim sensori
  Serial.println("║");
  Serial.printf("║ 🌡  BOSIM SENSORI (BMP280)          Holat: %s\n", data.bmpStatus.c_str());
  if (data.bmpOk) {
    Serial.printf("║    Harorat: %.2f °C\n", data.bmpTemp);
    Serial.printf("║    Bosim: %.2f hPa\n", data.pressure);
    Serial.printf("║    Balandlik: %.1f m\n", data.altitude);
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // DHT22 - Harorat/Namlik
  Serial.println("║");
  Serial.printf("║ 💧 HARORAT/NAMLIK (DHT22)           Holat: %s\n", data.dhtStatus.c_str());
  if (data.dhtOk) {
    Serial.printf("║    Harorat: %.2f °C\n", data.dhtTemp);
    Serial.printf("║    Namlik: %.2f %%\n", data.humidity);
    Serial.printf("║    Issiqlik indeksi: %.2f °C\n", data.heatIndex);
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // MQ135 - Gaz sensori
  Serial.println("║");
  Serial.printf("║ 🌫  GAZ SENSORI (MQ135)            Holat: %s\n", data.mq135Status.c_str());
  if (data.mq135Ok) {
    Serial.printf("║    Analog: %d  |  CO2: %.1f ppm\n", data.mq135Value, data.co2);
    Serial.printf("║    NH3: %.2f ppm  |  Benzol: %.2f ppm\n", data.nh3, data.benzene);
    Serial.printf("║    Havo sifati: %s\n", data.airQuality.c_str());
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // RadiationD-v1.1 - Geiger counter
  Serial.println("║");
  Serial.printf("║ ☢️  RADIATSIYA (Geiger Counter)     Holat: %s\n", data.geigerStatus.c_str());
  if (data.geigerOk) {
    Serial.printf("║    Impulslar: %.0f ta / daqiqa\n", data.cpm);
    Serial.printf("║    Doza tezligi: %.3f µSv/h\n", data.uSvh);
    Serial.printf("║    Daraja: %s\n", data.radiationLevel.c_str());
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // KY-037 - Ovoz sensori
  Serial.println("║");
  Serial.printf("║ 🔊 OVOZ SENSORI (KY-037)           Holat: %s\n", data.soundStatus.c_str());
  if (data.soundOk) {
    Serial.printf("║    Analog: %d  |  Shovqin: %.1f dB\n", data.soundAnalog, data.soundDB);
    Serial.printf("║    Daraja: %s\n", data.soundLevel.c_str());
    Serial.printf("║    Digital trigger: %s\n", data.soundDigital == LOW ? "AKTIV" : "PASSIV");
  } else {
    Serial.println("║    ⚠️  Sensor ulanmagan yoki topilmadi!");
  }
  
  // Xulosa va tavsiyalar
  Serial.println("║");
  Serial.println("╠════════════════════════════════════════════════════════════════════════════╣");
  Serial.println("║ 💡 TAVSIYALAR:");
  
  bool anyWarning = false;
  
  if (data.mq135Ok && data.co2 > 1000) {
    Serial.println("║    ⚠️  CO2 darajasi yuqori! Xonani ventilyatsiya qiling!");
    anyWarning = true;
  } else if (data.mq135Ok && data.co2 > 800) {
    Serial.println("║    ⚠️  CO2 darajasi o'rtachadan yuqori. Xonani shamollating.");
    anyWarning = true;
  }
  
  if (data.geigerOk && data.uSvh > 0.3) {
    Serial.println("║    ☢️  Radiatsiya darajasi yuqori! Ehtiyot bo'ling!");
    anyWarning = true;
  }
  
  if (data.soundOk && data.soundAnalog > SOUND_MEDIUM) {
    Serial.println("║    🔊 Shovqin juda baland! Quloq himoyasidan foydalaning!");
    anyWarning = true;
  }
  
  if (data.dhtOk && data.humidity > 70) {
    Serial.println("║    💧 Namlik yuqori! Namlagichdan foydalaning.");
    anyWarning = true;
  } else if (data.dhtOk && data.humidity < 30) {
    Serial.println("║    🔥 Namlik past! Namlagichdan foydalaning.");
    anyWarning = true;
  }
  
  if (!anyWarning) {
    Serial.println("║    ✅ Barcha ko'rsatkichlar normal. Davom eting!");
  }
  
  // Ulangan sensorlar soni
  int connected = 0;
  if (data.magOk) connected++;
  if (data.bmpOk) connected++;
  if (data.dhtOk) connected++;
  if (data.mq135Ok) connected++;
  if (data.geigerOk) connected++;
  if (data.soundOk) connected++;
  
  Serial.printf("║\n║ 📊 ULANGAN SENSORLAR: %d/6\n", connected);
  
  Serial.println("╚════════════════════════════════════════════════════════════════════════════╝\n");
}

// ==================== ASOSIY LOOP ====================
void loop() {
  unsigned long now = millis();
  
  // Har READ_INTERVAL vaqtda bir sensorlarni o'qish
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;
    readAllSensors();
    printAllData();
  }
  
  delay(10);
}
