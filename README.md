# GNSS Speed Tracker for ESP32-S3

A GNSS-based speed measurement and performance testing system built on the ESP32-S3 platform. This project features real-time speed tracking with Kalman filtering, a responsive web interface, and automated 0-to-target-speed timing capabilities.

![ESP32-S3 Speed Tracker](https://img.shields.io/badge/Platform-ESP32--S3-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)
![Framework](https://img.shields.io/badge/Framework-Arduino-green)

**⚠️ Notice**: This isn't the most accuarate thing ever since it does not integrate IMU, but my configuration was close enought (~100ms at worse of a actual timer- Dragy)

## The hardware I used

**Microcontroller**: [ESP32-S3 Super Mini](https://www.espboards.dev/esp32/esp32-s3-super-mini/)
- Ultra-compact board with 4MB Flash, 2MB PSRAM
- Castellated pins for easy integration
- Couple dollars

**GNSS Module**: Beitian BK-880Q
- capable of 25hz polling
- UBX protocol support
- Around 15-20 european money points

You dont have to use these specific boards to try it.

## Features

- **High-Precision GPS Tracking**: UBX protocol parser for u-blox GNSS modules (configurable update rate)
- **Kalman Filtering**: 1D Kalman filter for smooth, accurate speed estimation with GPS accuracy integration
- **Performance Testing**: Automated 0-to-target-speed timer with configurable target speeds
- **Real-Time Web Interface**: Responsive dashboard accessible via WiFi AP
- **Live Telemetry**: 
  - Current speed (km/h)
  - Elapsed time
  - GPS fix quality and satellite count
  - Filter confidence metrics
- **GPS Status Monitoring**: Visual indicators for satellite count and fix quality

## Hardware Requirements

### Required Components

- **ESP32-S3** Most ESP32 boards that supports wifi will work, this project used a esp32 s3 supermini 
- **u-blox GNSS Module**
  - Recommended: NEO-M9/NEO-M10 series, available cheaply as FPV drone modules (aim for 10+ hz polling rate)
  - Must support UBX NAV-PVT messages
- **GPS Antenna** (active antenna recommended for best performance)
- **Power Supply**: USB-C (5V) or external power source

### Pin Configuration

| Function | GPIO Pin |
|----------|----------|
| GPS RX   | GPIO 44  |
| GPS TX   | GPIO 43  |

**Note**: The GPS module's TX connects to ESP32's RX (GPIO 44), and GPS RX connects to ESP32's TX (GPIO 43).

### Wiring Diagram

```
ESP32-S3                    u-blox GNSS Module
---------                   ------------------
GPIO 44 (RX) <------------- TX
GPIO 43 (TX) -------------> RX
GND          <------------- GND
5V/3.3V      -------------> VCC
```

## 🚀 Quick Start

### 1. Prerequisites

- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- USB cable for programming
- GPS module with clear sky view for initial testing
- U-center for initial gnss module set-up

### 2. Clone the Repository

### 3. Build and Upload

#### Using PlatformIO (Recommended)

```bash
# Build the project
pio run

# Upload to ESP32-S3
pio run --target upload

# Monitor serial output
pio device monitor
```

#### Using Arduino IDE

1. Open `src/main.cpp` in Arduino IDE
2. Select **Board**: "ESP32S3 Dev Module"
3. Configure board settings:
   - Flash Size: 4MB (or whatever your board has)
   - PSRAM: "QSPI PSRAM" (or whatever your board has)
   - Partition Scheme: "Default 4MB with spiffs"
4. Select your COM port
5. Click **Upload**

### 4. Configure GNSS Module

Before first use, configure your u-blox GNSS module using **u-center** software ([download here](https://www.u-blox.com/en/product/u-center)).

#### Quick Configuration Steps

1. **Connect to module**
   - Open u-center → **Receiver → Port**
   - Select COM port, Baudrate: **9600** (default)

2. **Enable NAV-PVT message**
   ```
   View → Messages View → UBX → NAV → PVT
   - Check to enable
   - Right-click → Enable Message → Rate: 1 → Send
   ```

3. **Set update rate**
   ```
   View → Configuration View → RATE
   - Measurement Period: 50 ms (depends on used module)
   - Navigation Rate: 1
   - Click "Send"
   ```

4. **Disable NMEA messages**
   ```
   View → Configuration View → MSG
   - For each NMEA message (GGA, GLL, GSA, GSV, RMC, VTG):
     • Select message → Set UART1: 0 → Send
   - Verify NAV-PVT (01-07) is set to UART1: 1
   ```

5. **Configure UART to 230400 baud**
   ```
   View → Configuration View → PRT
   - Target: UART1
   - Baudrate: 230400 (going much lower with 25hz polling can cause issues, can be set higher if module allows)
   - Protocol In: UBX only
   - Protocol Out: UBX only (uncheck NMEA)
   - Click "Send"
   
   Immediately reconnect u-center:
   - Receiver → Port → Baudrate: 230400 → OK
   ```

6. **Save configuration**
   ```
   View → Configuration View → CFG
   - Check: BBR/Flash
   - Click "Send"
   ```

### 5. Connect to Web Interface

1. After upload, the ESP32-S3 creates a WiFi Access Point:
   - **SSID**: `ESP32-Performance-Tracker`
   - **Password**: None (open network)
2. Connect your phone/laptop to this network
3. Open a web browser and navigate to: `http://192.168.4.1`

## 📱 Using the Web Interface

### Performance Testing Mode

1. **ARM**: Click to prepare the system for timing
   - System must be stationary (< 2 km/h)
   - Status indicator turns yellow/amber
2. **Automatic Start**: Timer begins when motion is detected (> 3 km/h)
3. **Automatic Stop**: Timer stops when target speed is reached
4. **RESET**: Clear results and return to idle state

### Setting Target Speed

1. Enter desired speed in the input field (10-300 km/h)
2. Click "Set Target" button
3. Target is displayed in the stats grid

### Dashboard Elements

- **Speed Gauge**: Real-time animated circular gauge with color gradient
- **Status Badge**: Current system state (IDLE/ARMED/TIMING/FINISHED)
- **Stats Grid**:
  - Elapsed Time: Performance test duration
  - Target Speed: Configured target speed
  - Acceleration: Current acceleration in m/s²
  - Confidence: Kalman filter confidence (0-100%)
- **GPS Status Bar**: 
  - Green: Excellent (10+ satellites, 3D fix)
  - Yellow: Acceptable (6-9 satellites)
  - Red: Poor (< 6 satellites or no fix)

## ⚙️ Configuration

### GPS Settings

Edit these constants in `src/main.cpp`:

```cpp
#define GPS_RX_PIN 44              // ESP32 RX pin
#define GPS_TX_PIN 43              // ESP32 TX pin
const uint32_t GPS_BAUD_RATE = 230400;  // GPS baud rate
const uint16_t GNSS_UPDATE_INTERVAL_MS = 45; // Timing interval
```

### Performance Test Parameters

```cpp
const float STATIONARY_THRESHOLD_KMH = 2.0f;  // Speed considered stationary
const float START_TRIGGER_KMH = 3.0f;         // Speed to trigger timer
const float ACCEL_START_THRESHOLD = 0.8f;     // Acceleration trigger (m/s²)
```

### Kalman Filter Tuning

Adjust filter parameters in the `SimpleKalmanFilter` class:

```cpp
const float process_noise = 0.4f;              // Process noise
const float default_measurement_noise = 1.5f;  // GPS measurement noise
```

### WiFi Configuration

```cpp
WiFi.softAP("ESP32-Performance-Tracker");  // Change SSID here
```

To add a password:
```cpp
WiFi.softAP("ESP32-Performance-Tracker", "your_password");
```

## 🔧 Technical Details

### UBX Protocol Parser

The system implements a custom UBX protocol parser for u-blox GNSS modules:
- Processes **NAV-PVT** (Navigation Position Velocity Time) messages
- Extracts:
  - Ground speed (2D velocity)
  - 3D velocity components (North, East, Down)
  - Position (latitude, longitude, altitude)
  - Accuracy estimates (horizontal, vertical, speed)
  - Fix type and satellite count

### Kalman Filter Implementation

**1D Kalman Filter** optimized for speed estimation:
- **Prediction Step**: Optional acceleration-based prediction (currently disabled)
- **Update Step**: Incorporates GPS speed measurements with accuracy weighting
- **Adaptive Noise**: Process noise increases with acceleration
- **Timeout Handling**: Gentle decay during GPS signal loss
- **Stationary Detection**: Automatic zero-speed clamping when stationary

### Coordinate System

- **GPS Output**: WGS84 geodetic coordinates (latitude/longitude)
- **Velocity Frame**: NED (North-East-Down) reference frame
- **Local Conversion**: Included `gpsToLocal()` function for metric conversions

### Performance Characteristics

- **Update Rate**: ~50 Hz filter updates (20ms intervals)
- **GPS Processing**: Real-time, interrupt-driven UART parsing
- **Latency**: < 50ms from GPS measurement to web display
- **Timing Precision**: Microsecond-level timestamps for acceleration measurements

## 📊 Data Output

### JSON API Endpoint

Access raw data at `http://192.168.4.1/data`:

```json
{
  "s": 85.42,                    // Current speed (km/h)
  "t": 5.234,                    // Elapsed time (seconds)
  "st": "TIMING",                // State: IDLE/ARMED/TIMING/FINISHED
  "c": 0.87,                     // Filter confidence (0-1)
  "ax": 2.3,                     // Acceleration (m/s²)
  "ts": 100.0,                   // Target speed (km/h)
  "gps_fix": 3,                  // GPS fix type (0=none, 2=2D, 3=3D)
  "gps_sats": 12,                // Satellite count
  "gps_lat": 37.7749,            // Latitude (degrees)
  "gps_lon": -122.4194,          // Longitude (degrees)
  "gps_h_acc": 1500,             // Horizontal accuracy (mm)
  "gps_g_speed": 23728,          // GPS ground speed (mm/s)
  "filter_valid": true,          // Filter validity
  "gps_timeout": false           // GPS timeout status
}
```

## Dependencies

Automatically managed by PlatformIO:

- **espressif32** platform (ESP32 Arduino Core)
- **BasicLinearAlgebra** v5.1+ (matrix operations for future expansion)


## License

This project is MIT license.

