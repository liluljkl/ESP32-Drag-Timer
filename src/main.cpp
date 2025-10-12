#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <math.h>

// --- Hardware Configuration ---
#define GPS_SERIAL Serial2
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43

const uint32_t GPS_BAUD_RATE = 230400;
const uint16_t GNSS_UPDATE_INTERVAL_MS = 45;

// --- UBX Protocol Constants ---
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_MSG_NAV_PVT 0x07
#define UBX_NAV_PVT_LEN 92

// --- Simple 1D Kalman Filter for Speed ---
class SimpleKalmanFilter {
public:
  void initialize(float initial_speed_ms = 0.0f) {
    estimate_ms = initial_speed_ms;
    covariance = 1.0f;
  }

  void reset(float new_speed_ms) {
    estimate_ms = new_speed_ms;
    covariance = 1.0f;
  }

  void predict(float accel_ms2, float dt_s, bool use_accel) {
    if (dt_s <= 0.0f) {
      return;
    }

    if (use_accel) {
      estimate_ms += accel_ms2 * dt_s;
    }

    // Process noise grows with time and applied acceleration.
    float accel_term = fabsf(accel_ms2) * 0.05f;
    covariance += process_noise + accel_term;
  }

  void update(float measurement_ms, float measurement_noise) {
    float r = measurement_noise > 0.0f ? measurement_noise : default_measurement_noise;
    float innovation = measurement_ms - estimate_ms;
    float innovation_cov = covariance + r;
    float kalman_gain = (innovation_cov > 0.0f) ? covariance / innovation_cov : 0.0f;

    estimate_ms += kalman_gain * innovation;
    covariance = (1.0f - kalman_gain) * covariance;
    covariance = max(0.0001f, covariance);
  }

  float getSpeedMs() const { return estimate_ms; }
  float getCovariance() const { return covariance; }

private:
  float estimate_ms = 0.0f;
  float covariance = 1.0f;
  const float process_noise = 0.4f;
  const float default_measurement_noise = 1.5f;
};

struct SensorData {
  float gps_gnd_speed_ms = 0.0f;
  float gps_accuracy_ms = 5.0f;
  bool gps_valid = false;
  unsigned long timestamp = 0;
};

struct FilterOutput {
  float speed_ms = 0.0f;
  float speed_kmh = 0.0f;
  float acceleration = 0.0f;
  float confidence = 0.0f;
  bool stationary = true;
  bool valid = false;
};

SensorData sensorData;
FilterOutput filterOutput;

// --- UBX Parser (same as before) ---
enum UbxParseState {
  UBX_PARSE_SYNC1, UBX_PARSE_SYNC2, UBX_PARSE_CLASS, UBX_PARSE_ID,
  UBX_PARSE_LEN1, UBX_PARSE_LEN2, UBX_PARSE_PAYLOAD, UBX_PARSE_CK_A, UBX_PARSE_CK_B
};

struct UbxParser {
  UbxParseState state = UBX_PARSE_SYNC1;
  uint8_t msgClass = 0, msgId = 0;
  uint16_t payloadLen = 0, payloadPos = 0;
  uint8_t ckA = 0, ckB = 0;
  uint8_t payload[UBX_NAV_PVT_LEN];
};

struct GpsData {
  uint32_t iTOW = 0;
  uint8_t fixType = 0, numSV = 0;
  int32_t lon = 0, lat = 0, hMSL = 0;
  uint32_t hAcc = 0, vAcc = 0, sAcc = 0;
  int32_t velN = 0, velE = 0, velD = 0, gSpeed = 0;
  uint16_t pDOP = 0;
  uint8_t year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
  bool validTime = false, validDate = false, fullyResolved = false;
};

// --- Global Objects ---
SimpleKalmanFilter speedFilter;
UbxParser ubxParser;
GpsData gpsData;
WebServer server(80);

// --- Global State Flags ---
unsigned long lastValidGPS = 0; // For GPS timeout tracking, moved to global

// --- Performance Measurement ---
enum class PerformanceTestState : uint8_t { IDLE, ARMED, TIMING, FINISHED };
volatile PerformanceTestState currentState = PerformanceTestState::IDLE;

float targetSpeedKmh = 100.0f;
volatile float currentSpeedKmh = 0.0f;
unsigned long startTimeMicros = 0, stopTimeMicros = 0;
volatile float elapsedTimeSec = 0.0f;

const float STATIONARY_THRESHOLD_KMH = 2.0f;
const float START_TRIGGER_KMH = 3.0f;
const float ACCEL_START_THRESHOLD = 0.8f; // m/s²

// --- Sensor Data ---

// Reference position for relative positioning
double ref_lat = 0, ref_lon = 0;
bool ref_set = false;

// --- Utility Functions ---
void IRAM_ATTR calculateChecksum(uint8_t data, uint8_t &ckA, uint8_t &ckB) {
  ckA += data; ckB += ckA;
}

// Convert GPS coordinates to local meters
void gpsToLocal(double lat, double lon, float &x_local, float &y_local) { // Renamed x, y to avoid conflict
  if (!ref_set) {
    ref_lat = lat; ref_lon = lon; ref_set = true;
    x_local = y_local = 0; return;
  }
  // const double DEG_TO_RAD = PI / 180.0; // Remove this line, use DEG_TO_RAD from Arduino.h
  const double EARTH_RADIUS = 6378137.0;
  
  double dlat = (lat - ref_lat) * DEG_TO_RAD;
  double dlon = (lon - ref_lon) * DEG_TO_RAD * cos(ref_lat * DEG_TO_RAD);
  
  x_local = dlon * EARTH_RADIUS;
  y_local = dlat * EARTH_RADIUS;
}

bool IRAM_ATTR parseUbxByte(uint8_t data) {
  switch (ubxParser.state) {
    case UBX_PARSE_SYNC1:
      if (data == UBX_SYNC_CHAR_1) ubxParser.state = UBX_PARSE_SYNC2;
      break;
    case UBX_PARSE_SYNC2:
      if (data == UBX_SYNC_CHAR_2) {
        ubxParser.state = UBX_PARSE_CLASS;
        ubxParser.ckA = ubxParser.ckB = 0;
      } else ubxParser.state = UBX_PARSE_SYNC1;
      break;
    case UBX_PARSE_CLASS:
      ubxParser.msgClass = data;
      calculateChecksum(data, ubxParser.ckA, ubxParser.ckB);
      ubxParser.state = UBX_PARSE_ID;
      break;
    case UBX_PARSE_ID:
      ubxParser.msgId = data;
      calculateChecksum(data, ubxParser.ckA, ubxParser.ckB);
      ubxParser.state = UBX_PARSE_LEN1;
      break;
    case UBX_PARSE_LEN1:
      ubxParser.payloadLen = data;
      calculateChecksum(data, ubxParser.ckA, ubxParser.ckB);
      ubxParser.state = UBX_PARSE_LEN2;
      break;
    case UBX_PARSE_LEN2:
      ubxParser.payloadLen |= (data << 8);
      calculateChecksum(data, ubxParser.ckA, ubxParser.ckB);
      ubxParser.payloadPos = 0;
      ubxParser.state = (ubxParser.payloadLen == 0) ? UBX_PARSE_CK_A : UBX_PARSE_PAYLOAD;
      break;
    case UBX_PARSE_PAYLOAD:
      if (ubxParser.payloadPos < sizeof(ubxParser.payload)) {
        ubxParser.payload[ubxParser.payloadPos] = data;
      }
      calculateChecksum(data, ubxParser.ckA, ubxParser.ckB);
      ubxParser.payloadPos++;
      if (ubxParser.payloadPos >= ubxParser.payloadLen) {
        ubxParser.state = UBX_PARSE_CK_A;
      }
      break;
    case UBX_PARSE_CK_A:
      ubxParser.state = UBX_PARSE_SYNC1;
      if (data == ubxParser.ckA) ubxParser.state = UBX_PARSE_CK_B;
      break;
    case UBX_PARSE_CK_B:
      ubxParser.state = UBX_PARSE_SYNC1;
      return (data == ubxParser.ckB);
  }
  return false;
}

void IRAM_ATTR processUbxNavPvt() {
  if (ubxParser.payloadLen != UBX_NAV_PVT_LEN) {
    // Optional: Add a Serial.println("Error: NAV PVT payload length mismatch");
    sensorData.gps_valid = false; // Mark GPS data as invalid
    return; // Invalid payload length, abort processing
  }
  
  uint8_t *p = ubxParser.payload;
  
  gpsData.iTOW = *((uint32_t*)(p + 0));
  gpsData.fixType = p[20];
  gpsData.numSV = p[23];
  
  gpsData.lon = *((int32_t*)(p + 24));
  gpsData.lat = *((int32_t*)(p + 28));
  gpsData.hMSL = *((int32_t*)(p + 36)); // Height above mean sea level in mm
  gpsData.hAcc = *((uint32_t*)(p + 40));  // Horizontal accuracy estimate in mm
  gpsData.vAcc = *((uint32_t*)(p + 44));  // Vertical accuracy estimate in mm
  
  gpsData.velN = *((int32_t*)(p + 48));  // NED north velocity in mm/s
  gpsData.velE = *((int32_t*)(p + 52));  // NED east velocity in mm/s
  gpsData.velD = *((int32_t*)(p + 56));  // NED down velocity in mm/s
  gpsData.gSpeed = *((int32_t*)(p + 60)); // Ground Speed (2-D) in mm/s
  gpsData.sAcc = *((uint32_t*)(p + 68));  // Speed accuracy estimate in mm/s
    // Update sensor data for Kalman filter
  if (gpsData.fixType >= 2 && gpsData.fixType <= 4) { // Check for a valid fix (2D, 3D, GNSS+dead reckoning)
    sensorData.gps_valid = true;
    sensorData.gps_gnd_speed_ms = (float)gpsData.gSpeed / 1000.0f;
    float speed_accuracy_ms = max(0.2f, (float)gpsData.sAcc / 1000.0f);
    float horizontal_accuracy_ms = max(0.5f, (float)gpsData.hAcc / 1000.0f);
    // Use whichever is larger to stay conservative about speed confidence.
    sensorData.gps_accuracy_ms = max(speed_accuracy_ms, horizontal_accuracy_ms * 0.2f);
  } else {
    sensorData.gps_valid = false;
    sensorData.gps_gnd_speed_ms = 0.0f;
    sensorData.gps_accuracy_ms = 5.0f;
  }
}

void IRAM_ATTR processGpsChar(uint8_t c) {
  if (parseUbxByte(c)) {
    if (ubxParser.msgClass == UBX_CLASS_NAV && ubxParser.msgId == UBX_MSG_NAV_PVT) {
      processUbxNavPvt();
    }
  }
}

// --- Performance Measurement ---
void IRAM_ATTR handlePerformanceMeasurement() {
  if (!filterOutput.valid) return;
  
  // Enhanced trigger system using both GPS and accelerometer
  bool gps_trigger = (gpsData.fixType >= 2) &&
                     ((sensorData.gps_gnd_speed_ms * 3.6f) >= START_TRIGGER_KMH);
  bool accel_trigger = (fabsf(filterOutput.acceleration) >= ACCEL_START_THRESHOLD) &&
                       (filterOutput.confidence > 0.7f) && !filterOutput.stationary;
  
  switch (currentState) {
    case PerformanceTestState::IDLE:
      break;
      
    case PerformanceTestState::ARMED:
      if (gps_trigger || accel_trigger) {
        startTimeMicros = micros();
        currentState = PerformanceTestState::TIMING;
        elapsedTimeSec = 0.0f;
      }
      break;
      
    case PerformanceTestState::TIMING: {
      unsigned long currentMicros = micros();
      elapsedTimeSec = (currentMicros - startTimeMicros) / 1000000.0f;
      
      if (filterOutput.speed_kmh >= targetSpeedKmh) {
        stopTimeMicros = currentMicros;
        elapsedTimeSec = (stopTimeMicros - startTimeMicros) / 1000000.0f;
        currentState = PerformanceTestState::FINISHED;
      }
      break;
    }
    
    case PerformanceTestState::FINISHED:
      break;
  }
}

// --- Control Functions ---
void armSystem() {
  if (currentState == PerformanceTestState::IDLE || currentState == PerformanceTestState::FINISHED) {
    if (filterOutput.speed_kmh < STATIONARY_THRESHOLD_KMH) {
      currentState = PerformanceTestState::ARMED;
      elapsedTimeSec = 0.0f;
    }
  }
}

void resetSystem() {
  currentState = PerformanceTestState::IDLE;
  elapsedTimeSec = 0.0f;
  startTimeMicros = stopTimeMicros = 0;
}

void setTargetSpeed(float newTarget) {
  if (newTarget > START_TRIGGER_KMH && newTarget < 300.0f) {
    targetSpeedKmh = newTarget;
    if (currentState != PerformanceTestState::IDLE) resetSystem();
  }
}

// --- Web Interface ---
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-S3 Speed Tracker</title>
    <meta name='viewport' content='width=device-width,initial-scale=1'>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #fff;
            min-height: 100vh;
            padding: 20px;
        }
        .container { 
            max-width: 800px;
            margin: 0 auto;
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        }
        h1 { 
            text-align: center;
            font-size: 2.5em;
            margin-bottom: 30px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .gauge-container {
            position: relative;
            width: 300px;
            height: 300px;
            margin: 0 auto 30px;
        }
        #speedGauge {
            width: 100%;
            height: 100%;
        }
        .speed-display {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            text-align: center;
            pointer-events: none;
        }
        .speed-value {
            font-size: 3em;
            font-weight: bold;
            line-height: 1;
        }
        .speed-unit {
            font-size: 1.2em;
            opacity: 0.8;
        }
        .controls {
            display: flex;
            gap: 10px;
            justify-content: center;
            margin-bottom: 30px;
            flex-wrap: wrap;
        }
        button {
            background: rgba(255,255,255,0.2);
            color: white;
            border: 2px solid rgba(255,255,255,0.3);
            padding: 12px 24px;
            border-radius: 25px;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
            transition: all 0.3s;
            backdrop-filter: blur(5px);
        }
        button:hover {
            background: rgba(255,255,255,0.3);
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }
        button:active { transform: translateY(0); }
        button.armed { background: rgba(255,200,0,0.3); border-color: rgba(255,200,0,0.5); }
        button.timing { background: rgba(0,255,0,0.3); border-color: rgba(0,255,0,0.5); }
        button.finished { background: rgba(0,200,255,0.3); border-color: rgba(0,200,255,0.5); }
        input[type="number"] {
            background: rgba(255,255,255,0.2);
            border: 2px solid rgba(255,255,255,0.3);
            color: white;
            padding: 10px 15px;
            border-radius: 25px;
            font-size: 16px;
            width: 100px;
            text-align: center;
        }
        input[type="number"]::placeholder { color: rgba(255,255,255,0.6); }
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 20px;
        }
        .stat-card {
            background: rgba(255,255,255,0.15);
            padding: 15px;
            border-radius: 15px;
            text-align: center;
            transition: transform 0.2s;
        }
        .stat-card:hover { transform: translateY(-3px); }
        .stat-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .stat-value {
            font-size: 1.8em;
            font-weight: bold;
        }
        .status-indicator {
            display: inline-block;
            padding: 8px 20px;
            border-radius: 20px;
            font-weight: bold;
            margin-bottom: 20px;
        }
        .status-idle { background: rgba(150,150,150,0.3); }
        .status-armed { background: rgba(255,200,0,0.3); animation: pulse 1.5s infinite; }
        .status-timing { background: rgba(0,255,0,0.3); animation: pulse 1s infinite; }
        .status-finished { background: rgba(0,200,255,0.3); }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.6; }
        }
        .gps-status {
            text-align: center;
            padding: 10px;
            border-radius: 10px;
            margin-top: 15px;
            font-size: 0.9em;
        }
        .gps-good { background: rgba(0,255,0,0.2); }
        .gps-poor { background: rgba(255,200,0,0.2); }
        .gps-bad { background: rgba(255,0,0,0.2); }
    </style>
</head>
<body>
    <div class="container">
        <h1>Speed Tracker</h1>
        
        <div style="text-align: center;">
            <span id="statusBadge" class="status-indicator status-idle">IDLE</span>
        </div>

        <div class="gauge-container">
            <canvas id="speedGauge"></canvas>
            <div class="speed-display">
                <div class="speed-value" id="speedValue">0</div>
                <div class="speed-unit">km/h</div>
            </div>
        </div>

        <div class="controls">
            <button id="armBtn" onclick="armSystem()">ARM</button>
            <button onclick="resetSystem()">RESET</button>
            <input type="number" id="targetSpeedInput" value="100" step="10" min="10" max="300">
            <button onclick="setTargetSpeed()">Set Target</button>
        </div>

        <div class="stats-grid">
            <div class="stat-card">
                <div class="stat-label">Elapsed Time</div>
                <div class="stat-value" id="time">0.00s</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Target Speed</div>
                <div class="stat-value" id="targetDisplay">100</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Acceleration</div>
                <div class="stat-value" id="accel">0.0</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">Confidence</div>
                <div class="stat-value" id="conf">0%</div>
            </div>
        </div>

        <div id="gpsStatus" class="gps-status gps-bad">
            GPS: <span id="gpsSats">0</span> satellites | Fix: <span id="gpsFix">None</span>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('speedGauge');
        const ctx = canvas.getContext('2d');
        canvas.width = 300;
        canvas.height = 300;

        function drawGauge(speed, maxSpeed) {
            const centerX = 150, centerY = 150, radius = 120;
            ctx.clearRect(0, 0, 300, 300);
            
            // Background arc
            ctx.beginPath();
            ctx.arc(centerX, centerY, radius, 0.75 * Math.PI, 2.25 * Math.PI);
            ctx.lineWidth = 20;
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.stroke();
            
            // Speed arc
            const angle = 0.75 * Math.PI + (speed / maxSpeed) * 1.5 * Math.PI;
            ctx.beginPath();
            ctx.arc(centerX, centerY, radius, 0.75 * Math.PI, angle);
            ctx.lineWidth = 20;
            const gradient = ctx.createLinearGradient(0, 0, 300, 300);
            gradient.addColorStop(0, '#00ff88');
            gradient.addColorStop(0.5, '#ffaa00');
            gradient.addColorStop(1, '#ff0066');
            ctx.strokeStyle = gradient;
            ctx.lineCap = 'round';
            ctx.stroke();
            
            // Tick marks
            for (let i = 0; i <= maxSpeed; i += maxSpeed/10) {
                const tickAngle = 0.75 * Math.PI + (i / maxSpeed) * 1.5 * Math.PI;
                const x1 = centerX + Math.cos(tickAngle) * (radius - 25);
                const y1 = centerY + Math.sin(tickAngle) * (radius - 25);
                const x2 = centerX + Math.cos(tickAngle) * (radius - 15);
                const y2 = centerY + Math.sin(tickAngle) * (radius - 15);
                ctx.beginPath();
                ctx.moveTo(x1, y1);
                ctx.lineTo(x2, y2);
                ctx.strokeStyle = 'rgba(255,255,255,0.5)';
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }

        function setTargetSpeed() {
            const speed = document.getElementById('targetSpeedInput').value;
            fetch('/settarget?speed=' + speed)
                .then(response => response.text())
                .then(() => {
                    document.getElementById('targetDisplay').textContent = speed;
                });
        }

        function armSystem() {
            fetch('/arm');
        }

        function resetSystem() {
            fetch('/reset');
        }

        let currentSpeed = 0;
        let targetSpeed = 100;

        setInterval(() => {
            fetch('/data')
                .then(r => r.json())
                .then(d => {
                    targetSpeed = d.ts || 100;
                    currentSpeed = d.s;
                    
                    // Smooth speed animation
                    document.getElementById('speedValue').textContent = Math.round(currentSpeed);
                    drawGauge(currentSpeed, Math.max(targetSpeed * 1.2, 150));
                    
                    // Update status
                    const status = d.st;
                    const badge = document.getElementById('statusBadge');
                    badge.textContent = status;
                    badge.className = 'status-indicator status-' + status.toLowerCase();
                    
                    const armBtn = document.getElementById('armBtn');
                    armBtn.className = status === 'ARMED' ? 'armed' : 
                                       status === 'TIMING' ? 'timing' : 
                                       status === 'FINISHED' ? 'finished' : '';
                    
                    // Update stats
                    document.getElementById('time').textContent = d.t.toFixed(2) + 's';
                    document.getElementById('accel').textContent = d.ax.toFixed(1);
                    document.getElementById('conf').textContent = Math.round(d.c * 100) + '%';
                    document.getElementById('targetDisplay').textContent = Math.round(targetSpeed);
                    
                    // GPS status
                    const sats = d.gps_sats;
                    const gpsDiv = document.getElementById('gpsStatus');
                    document.getElementById('gpsSats').textContent = sats;
                    document.getElementById('gpsFix').textContent = d.gps_fix > 0 ? 'Active' : 'None';
                    
                    if (sats >= 10 && d.gps_fix >= 2) {
                        gpsDiv.className = 'gps-status gps-good';
                    } else if (sats >= 6) {
                        gpsDiv.className = 'gps-status gps-poor';
                    } else {
                        gpsDiv.className = 'gps-status gps-bad';
                    }
                })
                .catch(e => console.error("Error:", e));
        }, 100);
        
        drawGauge(0, 150);
    </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"s\":" + String(filterOutput.speed_kmh, 2);
  json += ",\"t\":" + String(elapsedTimeSec, 3);
  json += ",\"st\":\"";
  switch(currentState) {
    case PerformanceTestState::IDLE: json += "IDLE"; break;
    case PerformanceTestState::ARMED: json += "ARMED"; break;
    case PerformanceTestState::TIMING: json += "TIMING"; break;
    case PerformanceTestState::FINISHED: json += "FINISHED"; break;
  }
  json += "\"";
  json += ",\"c\":" + String(filterOutput.confidence, 2);
  json += ",\"ax\":" + String(filterOutput.acceleration, 2);
  json += ",\"ay\":0.00";
  json += ",\"ts\":" + String(targetSpeedKmh, 1); // Target Speed

  // GPS Data
  json += ",\"gps_fix\":" + String(gpsData.fixType);
  json += ",\"gps_sats\":" + String(gpsData.numSV);
  json += ",\"gps_lat\":" + String((double)gpsData.lat * 1e-7, 7);
  json += ",\"gps_lon\":" + String((double)gpsData.lon * 1e-7, 7);
  json += ",\"gps_h_acc\":" + String(gpsData.hAcc); // mm
  json += ",\"gps_g_speed\":" + String(gpsData.gSpeed); // mm/s
  json += ",\"gps_vel_n\":" + String(gpsData.velN); // mm/s
  json += ",\"gps_vel_e\":" + String(gpsData.velE); // mm/s
  
  // Raw IMU data for debugging
  json += ",\"raw_ax\":0.000";
  json += ",\"raw_ay\":0.000";
  json += ",\"raw_az\":0.000";
  json += ",\"imu_heading\":0.000";
    // Simple filter state debugging
  json += ",\"filter_valid\":" + String(filterOutput.valid ? "true" : "false");
  json += ",\"filter_vel_x\":" + String(filterOutput.speed_ms, 2); // legacy field name for UI
  json += ",\"filter_vel_y\":0";
  json += ",\"gps_valid\":" + String(((lastValidGPS > 0) && ((micros() - lastValidGPS) < 500000)) ? "true" : "false");
  
  // GPS timeout status
  unsigned long gps_timeout_ms = lastValidGPS > 0 ? (micros() - lastValidGPS) / 1000 : 0;
  json += ",\"gps_timeout_ms\":" + String(gps_timeout_ms);
  json += ",\"gps_timeout\":" + String(gps_timeout_ms > 2000 ? "true" : "false");
  
  // Check if state changed to include in log
  static PerformanceTestState lastReportedState = currentState;
  if (currentState != lastReportedState) {
    json += ",\"st_changed\":true";
    lastReportedState = currentState;
  } else {
    json += ",\"st_changed\":false";
  }

  json += "}";
  
  server.send(200, "application/json", json);
}

void handleArm() { 
  armSystem(); 
  server.send(200, "text/plain", "System Armed"); 
}
void handleReset() { 
  resetSystem(); 
  server.send(200, "text/plain", "System Reset"); 
}

// New handler for setting target speed
void handleSetTarget() {
  if (server.hasArg("speed")) {
    float newSpeed = server.arg("speed").toFloat();
    if (newSpeed > 0 && newSpeed < 500) { // Basic validation
      setTargetSpeed(newSpeed);
      server.send(200, "text/plain", "Target speed set to " + String(newSpeed));
      return;
    }
  }
  server.send(400, "text/plain", "Invalid speed value");
}

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Start with LED off
  
  // Initialize GPS
  GPS_SERIAL.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize simple speed filter
  speedFilter.initialize(0.0f);

  // --- Initialize WiFi AP and print its IP ---
  WiFi.softAP("ESP32-Performance-Tracker");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // --- Web Server Routes ---
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/arm", handleArm);
  server.on("/reset", handleReset);
  server.on("/settarget", handleSetTarget); // Add new route
  server.begin();
  
  Serial.println("System initialized - Ready for performance measurement!");
}

// --- Main Loop ---
void loop() {
  // Blink the built-in LED every second for visual feedback
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // toggle LED
  }

  static unsigned long lastFilterUpdate = 0;
  
  unsigned long currentTime = micros();
  
  // Process GPS data
  while (GPS_SERIAL.available()) {
    processGpsChar(GPS_SERIAL.read());
  }
  
  // Simple 1D Kalman filter update logic
  const unsigned long GPS_TIMEOUT_US = 2000000; // 2 seconds GPS timeout

  if (sensorData.gps_valid) {
    lastValidGPS = currentTime;
  }

  bool gps_timeout = (lastValidGPS > 0) && ((currentTime - lastValidGPS) > GPS_TIMEOUT_US);
  const unsigned long update_interval_us = 20000; // ~50 Hz update cadence without IMU support

  if ((currentTime - lastFilterUpdate) >= update_interval_us) {
    float dt_sec = (lastFilterUpdate == 0) ? (update_interval_us / 1000000.0f)
                                           : (currentTime - lastFilterUpdate) / 1000000.0f;
    dt_sec = constrain(dt_sec, 0.001f, 0.2f);

    float prev_speed_ms = speedFilter.getSpeedMs();

    bool new_gps = sensorData.gps_valid;

    // Determine stationarity based purely on recent speed levels
    float reference_speed_ms = new_gps ? sensorData.gps_gnd_speed_ms : prev_speed_ms;
    bool stationary = reference_speed_ms < 0.3f;

    speedFilter.predict(0.0f, dt_sec, false);

    if (new_gps) {
      speedFilter.update(sensorData.gps_gnd_speed_ms, sensorData.gps_accuracy_ms);
    } else if (stationary) {
      speedFilter.update(0.0f, 1.5f); // bias estimate gently toward zero when stationary without GPS
    }

    float estimated_speed_ms = max(0.0f, speedFilter.getSpeedMs());

    if (stationary && estimated_speed_ms < 0.25f) {
      estimated_speed_ms = 0.0f;
      speedFilter.reset(0.0f);
    } else if (gps_timeout && !new_gps) {
      float decay_factor = max(0.0f, 1.0f - (dt_sec * 0.05f)); // gently decay ~5%/s during GPS gaps
      float decayed_speed = estimated_speed_ms * decay_factor;
      estimated_speed_ms = decayed_speed;
      speedFilter.reset(decayed_speed);
    }

    float acceleration_ms2 = dt_sec > 0.0f ? (estimated_speed_ms - prev_speed_ms) / dt_sec : 0.0f;

    filterOutput.speed_ms = estimated_speed_ms;
    filterOutput.speed_kmh = estimated_speed_ms * 3.6f;
    filterOutput.acceleration = acceleration_ms2;
    filterOutput.confidence = 1.0f / (1.0f + speedFilter.getCovariance());
    filterOutput.stationary = stationary;
    filterOutput.valid = true;

    currentSpeedKmh = filterOutput.speed_kmh;

    sensorData.gps_valid = false;
    lastFilterUpdate = currentTime;

    handlePerformanceMeasurement();
  }
  
  server.handleClient();
  if (currentTime % 10000 == 0) yield(); // Yield every 10ms to prevent watchdog
}

// --- Simple speed filter loop end ---