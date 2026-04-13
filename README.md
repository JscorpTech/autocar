# 🚗 Avtonom Mashina Robot

Raspberry Pi + ESP32 asosida avtonom harakatlanadigan Ackermann tuzilishidagi mashina robot.
Mashinaga xarita (JSON) beriladi va u **A\* algoritmi** bilan yo'l topib, **encoder odometriyasi** yordamida mustaqil harakatlanadi.

## Arxitektura

```
┌─────────────────┐    UART Serial    ┌─────────────────┐
│   Raspberry Pi  │◄────────────────►│     ESP32       │
│                 │   CMD/DATA        │                 │
│  • Xarita       │                   │  • Haydash      │
│  • A* yo'l      │                   │    motorlari    │
│  • Navigatsiya  │                   │  • Rul motori   │
│  • PID          │                   │  • 4x Encoder   │
│  • Web UI       │                   │  • 6x Ultrasonic│
└─────────────────┘                   └─────────────────┘
```

## Qurilma Komponentlari

| Komponent | Miqdor | Vazifa |
|-----------|--------|--------|
| Raspberry Pi 3/4/5 | 1 | Bosh kompyuter — yo'l topish, navigatsiya, Web UI |
| ESP32 | 1 | Motor boshqaruv, sensor o'qish, odometriya |
| BTS7960 Motor Driver | 2 | Oldinga/orqaga haydash motorlarini boshqarish |
| L298N Motor Driver | 1 | Rul motorini boshqarish |
| LM393 IR Encoder | 4 | G'ildirak aylanishini hisoblash (odometriya) |
| HC-SR04 Ultrasonic | 6 | To'siq aniqlash (old, old-o'ng, old-chap, o'ng, chap, orqa) |

## Pin Ulanishlari (ESP32)

### Haydash (BTS7960) va Rul (L298N)

| Pin | Signal | Tavsif |
|-----|--------|--------|
| GPIO 18 | FRONT_RPWM | Old motor — oldinga PWM |
| GPIO 19 | FRONT_LPWM | Old motor — orqaga PWM |
| GPIO 21 | REAR_RPWM  | Orqa motor — oldinga PWM |
| GPIO 22 | REAR_LPWM  | Orqa motor — orqaga PWM |
| GPIO 26 | STEER_IN1 | L298N rul kanali A IN1 |
| GPIO 27 | STEER_IN2 | L298N rul kanali A IN2 |
| GPIO 12 | STEER_IN3 | L298N rul kanali B IN3 |
| GPIO 25 | STEER_IN4 | L298N rul kanali B IN4 |

`L298N ENA` va `ENB` pinlari doimiy `HIGH` (3.3V) holatda ishlatiladi.

### Ultrasonic Sensorlar (HC-SR04)

| TRIG | ECHO | Sensor |
|------|------|--------|
| GPIO 13 | GPIO 14 | Old markaziy |
| GPIO 15 | GPIO 16 | Old o'ng |
| GPIO 17 | GPIO 23 | Old chap |
| GPIO 41 | GPIO 32 | O'ng yon |
| GPIO 33 | GPIO 34 | Chap yon |
| GPIO 42 | GPIO 35 | Orqa |

### Encoder Sensorlar (LM393 IR)

| Pin | Tavsif |
|-----|--------|
| GPIO 36 | Old o'ng encoder (faqat kirish, pull-up yo'q) |
| GPIO 39 | Old chap encoder (faqat kirish, pull-up yo'q) |
| GPIO 0  | Orqa o'ng encoder (strapping pin, INPUT_PULLUP) |
| GPIO 2  | Orqa chap encoder (strapping pin, INPUT_PULLUP) |

### Serial Aloqa

| Pin | Signal |
|-----|--------|
| GPIO 43 | TX — Raspberry Pi ga |
| GPIO 44 | RX — Raspberry Pi dan |

## O'rnatish

### ESP32
1. Arduino IDE yoki PlatformIO o'rnating
2. ESP32 board paketini qo'shing (`esp32 by Espressif Systems`)
3. Qo'shimcha kutubxona kerak emas — standart Arduino API ishlatiladi
4. `esp32/car_controller/car_controller.ino` faylini ESP32 ga yuklang

### Raspberry Pi
```bash
# Python kutubxonalarini o'rnating
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

Tayyor xaritalar `maps/` papkasida: `example_map.json` (10×10 labirint), `simple_straight.json` (5×10 to'g'ri yo'l).

### 2. Terminal orqali ishlatish
```bash
cd raspberry_pi

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
  AUTONOMOUS CAR
==================================================

Connecting to ESP32...
[COMM] ESP32 connected: /dev/ttyUSB0

map: ../maps/example_map.json
[MAP] loaded: 10x10, start=(0,0), end=(9,9)

[MAP] path found, 28 steps
[MAP] 7 waypoints

press Enter when ready >>>

navigation started!
[NAV] waypoint 1/7: 90.0°, 3.0m
  [DRIVE] 3.00m traveled
...
[NAV] destination reached!
```

## Aloqa Protokoli

### Pi → ESP32 (Buyruqlar)

| Buyruq | Format | Tavsif |
|--------|--------|--------|
| Haydash | `CMD:speed,steer_angle\n` | Tezlik −255..255, rul burchagi −30..+30° |
| To'xtatish | `STOP\n` | Barcha motorlarni to'xtatish |
| Ping | `PING\n` | Aloqa tekshirish (`PONG` javob kutiladi) |
| Encoder reset | `RESET_ENC\n` | Encoderlar va heading ni nolga qaytarish |

### ESP32 → Pi (Telemetriya)

| Xabar | Format | Tavsif |
|-------|--------|--------|
| Ma'lumot | `DATA:encL,encR,heading,rpmL,rpmR,dFront,dFrontRight,dFrontLeft,dRight,dLeft,dRear\n` | 10 Hz |
| Old to'siq | `WARN:OBSTACLE\n` | Oldinda to'siq ≤ 0.5 m |
| Orqa to'siq | `WARN:OBSTACLE_REAR\n` | Orqada to'siq ≤ 0.5 m |
| Timeout | `WARN:TIMEOUT\n` | 2 soniya buyruq kelmadi |

**DATA maydoni izohi:**

| Maydon | Birlik | Tavsif |
|--------|--------|--------|
| `encL` | puls | Chap encoder umumiy pulslari (old+orqa o'rtacha) |
| `encR` | puls | O'ng encoder umumiy pulslari (old+orqa o'rtacha) |
| `heading` | daraja | Odometriya asosida yo'nalish (shimol = 0°) |
| `rpmL` | RPM | Chap g'ildirak aylanish tezligi |
| `rpmR` | RPM | O'ng g'ildirak aylanish tezligi |
| `dFront` | metr | Old markaziy sensor masofasi |
| `dFrontRight` | metr | Old o'ng sensor masofasi |
| `dFrontLeft` | metr | Old chap sensor masofasi |
| `dRight` | metr | O'ng yon sensor masofasi |
| `dLeft` | metr | Chap yon sensor masofasi |
| `dRear` | metr | Orqa sensor masofasi |

## Sozlash

`raspberry_pi/config.py` faylida barcha parametrlarni o'zgartirish mumkin:

| Parametr | Standart | Tavsif |
|----------|----------|--------|
| `SERIAL_PORT` | `/dev/ttyUSB0` | ESP32 serial porti |
| `SERIAL_BAUD` | `115200` | Baud rate |
| `WHEEL_DIAMETER` | `0.25 m` | G'ildirak diametri |
| `WHEEL_BASE` | `1.8 m` | Old va orqa o'q orasidagi masofa |
| `PULSES_PER_REV` | `4` | Bir aylanishdagi encoder pulslar soni |
| `MAX_STEER_ANGLE` | `30°` | Maksimal rul burchagi |
| `BASE_SPEED` | `150` | Oddiy haydash tezligi (PWM 0–255) |
| `OBSTACLE_DISTANCE` | `0.5 m` | To'siq to'xtatish masofasi |
| `PID_KP/KI/KD` | `1.5 / 0.03 / 0.4` | To'g'ri haydash PID koeffitsientlari |
| `TURN_PID_KP/KI/KD` | `1.0 / 0.01 / 0.3` | Burilish PID koeffitsientlari |

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
| 🧭 **Yo'nalish** | Real-time odometriya asosida heading ko'rsatish |
| 📊 **Telemetriya** | Encoder, RPM, 6 ta sensor masofasi — real-time yangilanadi |
| 🛡️ **Xavfsizlik** | To'siq indikatori, favqulodda to'xtatish (Spacebar) |
| 📋 **Jurnal** | Barcha hodisalar rangli log panelda ko'rinadi |

### Klaviatura boshqaruv

| Tugma | Harakat |
|-------|---------|
| `W` / `↑` | Oldinga |
| `S` / `↓` | Orqaga |
| `A` / `←` | Chapga (rul) |
| `D` / `→` | O'ngga (rul) |
| `Probel` | Favqulodda to'xtatish |

## Xavfsizlik

- **6 ta to'siq sensori**: Old (markaziy, o'ng, chap), yon (o'ng, chap), orqa — 0.5 m dan yaqin to'siq aniqlansa mashina to'xtaydi
- **Orqa to'siq**: Orqaga ketayotganda orqa sensor nazorat qilinadi
- **3-nuqtali burilish**: Tor joyda burilib bo'lmasa, mashina orqaga chekinib qayta urinadi (5 marta)
- **Buyruq timeout**: 2 sekund ichida Pi dan buyruq kelmasa motorlar avtomatik to'xtaydi
- **Ctrl+C / Spacebar**: Istalgan vaqtda to'xtatish mumkin
- **Favqulodda to'xtatish**: Web UI dagi qizil tugma yoki Spacebar
