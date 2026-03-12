# 🚗 Avtonom Mashina Robot

Raspberry Pi + ESP32-S3 asosida avtonom harakatlanadigan mashina robot.
Mashinaga xarita (JSON) beriladi va u **A\* algoritmi** bilan yo'l topib, **encoder + kompas** yordamida mustaqil harakatlanadi.

## Arxitektura

```
┌─────────────────┐    UART Serial    ┌─────────────────┐
│   Raspberry Pi  │◄────────────────►│    ESP32-S3     │
│                 │   CMD/DATA        │                 │
│  • Xarita       │                   │  • Motorlar     │
│  • A* yo'l      │                   │  • Encoderlar   │
│  • Navigatsiya  │                   │  • Kompas       │
│  • PID          │                   │  • Ultrasonic   │
└─────────────────┘                   └─────────────────┘
```

## Qurilma Komponentlari

| Komponent | Vazifa |
|-----------|--------|
| Raspberry Pi 3/4/5 | Bosh kompyuter - yo'l topish va navigatsiya |
| ESP32-S3 | Motor boshqaruv, sensor o'qish |
| L298N Motor Driver | DC motorlarni boshqarish |
| LM393 IR Encoder x2 | G'ildirak aylanishini hisoblash |
| HMC5883L | Kompas - yo'nalish aniqlash |
| HC-SR04 | Ultrasonic - to'siq aniqlash |

## Pin Ulanishlari (ESP32-S3)

| Pin | Komponent |
|-----|-----------|
| GPIO 4 | Encoder chap (LM393) |
| GPIO 5 | Encoder o'ng (LM393) |
| GPIO 8 | I2C SDA (HMC5883L) |
| GPIO 9 | I2C SCL (HMC5883L) |
| GPIO 10 | Ultrasonic TRIG |
| GPIO 11 | Ultrasonic ECHO |
| GPIO 16 | Motor A IN1 |
| GPIO 17 | Motor A IN2 |
| GPIO 18 | Motor A ENA (PWM) |
| GPIO 19 | Motor B IN3 |
| GPIO 20 | Motor B IN4 |
| GPIO 21 | Motor B ENB (PWM) |
| GPIO 43 | Serial TX (Pi ga) |
| GPIO 44 | Serial RX (Pi dan) |

## O'rnatish

### ESP32-S3
1. Arduino IDE yoki PlatformIO o'rnating
2. ESP32 board paketini qo'shing
3. Kutubxonalarni o'rnating:
   - `Adafruit HMC5883 Unified`
   - `Adafruit Unified Sensor`
4. `esp32/car_controller/car_controller.ino` faylini ESP32 ga yuklang

### Raspberry Pi
```bash
# Python kutubxonalarni o'rnating
pip3 install pyserial flask flask-socketio

# Loyiha papkasiga o'ting
cd raspberry_pi
```

## Ishlatish

### 1. Xarita yaratish (JSON)
```json
{
  "name": "Mening xaritam",
  "cell_size": 1.0,
  "start": [0, 0],
  "end": [4, 4],
  "map": [
    [1, 1, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1]
  ]
}
```
- `1` = yo'l (yursa bo'ladi)
- `0` = devor/to'siq
- `start` = boshlang'ich nuqta `[row, col]`
- `end` = manzil nuqta `[row, col]`
- `cell_size` = har bir katak necha metr

### 2. Dasturni ishga tushirish
```bash
# Haqiqiy mashina bilan
python3 main.py ../maps/example_map.json

# Serial portni ko'rsatish
python3 main.py ../maps/example_map.json --port /dev/ttyUSB0

# Simulyatsiya (ESP32 siz test)
python3 main.py --simulate ../maps/example_map.json
```

### 3. Natija
```
==================================================
  AVTONOM MASHINA ROBOT
==================================================

[1] ESP32 ga ulanish...
[COMM] ESP32 ga ulandi: /dev/ttyUSB0

[2] Xarita yuklanmoqda...
[MAP] Xarita yuklandi: 10x10

[XARITA]
 S . . # # # . . . .
 # # . # . # . # # .
 . # . . . # . # # .
 . # # # . # . . # .
 . . . # . # # . # .
 # # . # . . . . # .
 # # . # # # # . # .
 . . . . . # # . . .
 . # # # . # # # # #
 . . . . . . . . . E

[3] Yo'l topilmoqda (A* algoritmi)...
[MAP] Yo'l topildi! 28 qadam.

[5] Navigatsiya boshlandi!
  [TURN] Maqsad yo'nalish: 90°
  [DRIVE] Masofa: 40 sm
  ...
✅ MANZILGA MUVAFFAQIYATLI YETILDI!
```

## Aloqa Protokoli

### Pi → ESP32 (Buyruqlar)
| Buyruq | Format | Tavsif |
|--------|--------|--------|
| Motor | `CMD:left,right\n` | Tezlik -255..255 |
| To'xtatish | `STOP\n` | Motorlarni to'xtatish |
| Ping | `PING\n` | Aloqa tekshirish |
| Encoder reset | `RESET_ENC\n` | Encoderlarni nolga |

### ESP32 → Pi (Telemetriya)
| Xabar | Format | Tavsif |
|-------|--------|--------|
| Ma'lumot | `DATA:encL,encR,heading,rpmL,rpmR,dist\n` | 10 Hz |
| Ogohlantirish | `WARN:OBSTACLE\n` | To'siq aniqlandi |
| Ogohlantirish | `WARN:TIMEOUT\n` | Buyruq kelmadi |

## Sozlash

`raspberry_pi/config.py` faylida barcha parametrlarni o'zgartirish mumkin:
- G'ildirak o'lchamlari
- PID koeffitsientlari
- Motor tezliklari
- Xavfsizlik masofasi

## 🖥️ Web UI Dashboard

Professional web-interfeys orqali mashinani real-time boshqarish va monitoring qilish.

### Ishga tushirish
```bash
cd raspberry_pi

# Simulyatsiya rejimi (ESP32 siz test)
python3 web_ui.py --simulate

# Haqiqiy qurilma bilan
python3 web_ui.py --port /dev/ttyUSB0

# Boshqa portda ishga tushirish
python3 web_ui.py --simulate --web-port 8080
```

Brauzerda oching: `http://localhost:5000` (yoki `--web-port` bilan ko'rsatilgan port)

### Imkoniyatlar

| Bo'lim | Tavsif |
|--------|--------|
| 🗺️ **Xarita paneli** | Xarita tanlash, yuklash, yo'l topish (A*), canvas vizualizatsiya |
| ▶️ **Navigatsiya** | Start/Stop/Favqulodda to'xtatish tugmalari, waypoint progress bar |
| 🎮 **Qo'lda boshqarish** | D-pad tugmalar + klaviatura (WASD/strelkalar), tezlik slider |
| 🧭 **Kompas** | Real-time yo'nalish ko'rsatish (needle animatsiya) |
| 📊 **Telemetriya** | Encoder, RPM, to'siq masofasi — real-time yangilanadi |
| 🛡️ **Xavfsizlik** | To'siq indikatori, favqulodda to'xtatish (Spacebar) |
| 📋 **Jurnal** | Barcha hodisalar rangli log panelda ko'rinadi |

### Klaviatura boshqaruv

| Tugma | Harakat |
|-------|---------|
| `W` / `↑` | Oldinga |
| `S` / `↓` | Orqaga |
| `A` / `←` | Chapga |
| `D` / `→` | O'ngga |
| `Probel` | Favqulodda to'xtatish |

### Xarita formati
```json
{
  "name": "Mening xaritam",
  "cell_size": 1.0,
  "start": [0, 0],
  "end": [4, 4],
  "map": [
    [1, 1, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1]
  ]
}
```

Xaritani `maps/` papkasiga `.json` sifatida joylashtiring yoki Web UI orqali yuklang.

## Xavfsizlik

- **To'siq aniqlash**: Ultrasonic sensor 15sm dan yaqin to'siq aniqlasa mashina to'xtaydi
- **Buyruq timeout**: 2 sekund ichida Pi dan buyruq kelmasa motorlar to'xtaydi
- **Ctrl+C / Spacebar**: Istalgan vaqtda to'xtatish mumkin
- **Favqulodda to'xtatish**: Web UI dagi qizil tugma yoki Spacebar
