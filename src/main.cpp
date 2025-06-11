#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// --- Hardware Configuration ---
#define GPS_SERIAL Serial2
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
#define IMU_SDA_PIN 8
#define IMU_SCL_PIN 9
#define IMU_INT1_PIN 10
#define IMU_INT2_PIN 11

const uint32_t GPS_BAUD_RATE = 230400;
const uint16_t GNSS_UPDATE_INTERVAL_MS = 45;

// --- UBX Protocol Constants ---
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_MSG_NAV_PVT 0x07
#define UBX_NAV_PVT_LEN 92

// --- Kalman Filter Implementation ---
#define STATE_SIZE 6    // [pos_x, pos_y, vel_x, vel_y, accel_x, accel_y]
#define GPS_MEAS_SIZE 4 // [pos_x, pos_y, vel_x, vel_y]
#define IMU_MEAS_SIZE 2 // [accel_x, accel_y]

class AdvancedKalmanFilter {
private:
  Matrix<STATE_SIZE, 1> x;                    // State vector
  Matrix<STATE_SIZE, STATE_SIZE> P;           // Covariance matrix
  Matrix<STATE_SIZE, STATE_SIZE> F_kf;        // State transition (Renamed from F)
  Matrix<STATE_SIZE, STATE_SIZE> Q;           // Process noise
  Matrix<GPS_MEAS_SIZE, STATE_SIZE> H_gps;    // GPS observation matrix
  Matrix<IMU_MEAS_SIZE, STATE_SIZE> H_imu;    // IMU observation matrix
  Matrix<GPS_MEAS_SIZE, GPS_MEAS_SIZE> R_gps; // GPS measurement noise
  Matrix<IMU_MEAS_SIZE, IMU_MEAS_SIZE> R_imu; // IMU measurement noise
  
  float dt = 0.045f; // 45ms GPS update rate
  unsigned long lastUpdate = 0;
  
  // Bias estimation
  float accel_bias_x = 0.0f;
  float accel_bias_y = 0.0f;
  float gyro_bias_z = 0.0f;
  
public:
  // Make bias values accessible for calibration
  float& getAccelBiasX() { return accel_bias_x; }
  float& getAccelBiasY() { return accel_bias_y; }
  float& getGyroBiasZ() { return gyro_bias_z; }
  
  struct SensorData {
    // GPS data
    float gps_pos_x = 0, gps_pos_y = 0;
    float gps_vel_x = 0, gps_vel_y = 0;
    float gps_vel_n = 0, gps_vel_e = 0, gps_vel_d = 0; // Added NED velocities
    float gps_gnd_speed = 0; // Added ground speed
    float gps_sAcc = 0;      // Added speed accuracy
    float gps_hAcc = 0;      // Added horizontal accuracy
    float gps_accuracy = 100;
    bool gps_valid = false;
    
    // IMU data (world frame)
    float accel_x = 0, accel_y = 0, accel_z = 0;
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    float heading = 0, pitch = 0, roll = 0;
    
    unsigned long timestamp = 0;
  };
  
  struct FilterOutput {
    float position_x = 0, position_y = 0;
    float velocity_x = 0, velocity_y = 0;
    float acceleration_x = 0, acceleration_y = 0;
    float speed_ms = 0, speed_kmh = 0;
    float heading = 0;
    float confidence = 0;
    bool valid = false;
  };
  
  void initialize() {
    // Initialize state [pos_x, pos_y, vel_x, vel_y, accel_x, accel_y]
    x.Fill(0);
    
    // Initialize covariance with high uncertainty
    P.Fill(0);
    P(0,0) = 100;   // position uncertainty (m²)
    P(1,1) = 100;
    P(2,2) = 50;    // Changed from 25 - increased initial velocity uncertainty
    P(3,3) = 50;    // Changed from 25
    P(4,4) = 4;     // acceleration uncertainty (m²/s⁴)
    P(5,5) = 4;
    
    // Process noise
    Q.Fill(0);
    float sigma_a = 1.0f; // Lowered from 2.0 - reduced acceleration noise
    float initial_dt = 0.045f;
    Q(0,0) = pow(initial_dt,4)/4 * sigma_a;
    Q(1,1) = pow(initial_dt,4)/4 * sigma_a;
    Q(2,2) = pow(initial_dt,2) * sigma_a;
    Q(3,3) = pow(initial_dt,2) * sigma_a;
    Q(4,4) = sigma_a;
    Q(5,5) = sigma_a;
    
    // GPS measurement noise
    R_gps.Fill(0);
    R_gps(0,0) = 25;   // GPS position noise (5m std)
    R_gps(1,1) = 25;
    R_gps(2,2) = 1;    // GPS velocity noise (1 m/s std)
    R_gps(3,3) = 1;
    
    // IMU measurement noise
    R_imu.Fill(0);
    R_imu(0,0) = 0.04; // Accelerometer noise (0.2 m/s² std)
    R_imu(1,1) = 0.04;
    
    // GPS observation matrix [pos_x, pos_y, vel_x, vel_y]
    H_gps.Fill(0);
    H_gps(0,0) = 1; // GPS measures position directly
    H_gps(1,1) = 1;
    H_gps(2,2) = 1; // GPS measures velocity directly
    H_gps(3,3) = 1;
    
    // IMU observation matrix [accel_x, accel_y]
    H_imu.Fill(0);
    H_imu(0,4) = 1; // IMU measures acceleration directly
    H_imu(1,5) = 1;
    
    lastUpdate = micros();
  }
    void predict(float delta_time) {
    dt = delta_time;
    
    // Update state transition matrix
    F_kf.Fill(0); // Use renamed F_kf
    // Position evolves with velocity and acceleration
    F_kf(0,0) = 1; F_kf(0,2) = dt; F_kf(0,4) = 0.5*dt*dt;
    F_kf(1,1) = 1; F_kf(1,3) = dt; F_kf(1,5) = 0.5*dt*dt;
    // Velocity evolves with acceleration
    F_kf(2,2) = 1; F_kf(2,4) = dt;
    F_kf(3,3) = 1; F_kf(3,5) = dt;
    // Acceleration is constant
    F_kf(4,4) = 1;
    F_kf(5,5) = 1;
    
    // Predict state
    x = F_kf * x;
    
    // Add velocity damping
    x(2) *= 0.995; // 0.5% damping per prediction step
    x(3) *= 0.995;
    
    // Adaptive process noise based on current acceleration magnitude
    float current_accel = sqrt(x(4)*x(4) + x(5)*x(5));
    float sigma_a = 2.0f; // Base acceleration noise
    if (current_accel > 3.0f) {
      sigma_a = 4.0f; // Higher noise during high acceleration
    } else if (current_accel < 0.5f) {
      sigma_a = 1.0f; // Lower noise during stationary periods
    }
    
    // Update process noise matrix with current dt and adaptive sigma
    Matrix<STATE_SIZE, STATE_SIZE> Q_adaptive = Q;
    Q_adaptive(0,0) = pow(dt,4)/4 * sigma_a;
    Q_adaptive(1,1) = pow(dt,4)/4 * sigma_a;
    Q_adaptive(2,2) = pow(dt,2) * sigma_a;
    Q_adaptive(3,3) = pow(dt,2) * sigma_a;
    Q_adaptive(4,4) = sigma_a;
    Q_adaptive(5,5) = sigma_a;
    
    // Predict covariance
    P = F_kf * P * (~F_kf) + Q_adaptive; // Use adaptive Q
  }
  
  void updateGPS(const SensorData& data) {
    if (!data.gps_valid) return;
    
    // Create GPS measurement vector
    Matrix<GPS_MEAS_SIZE, 1> z_gps;
    z_gps(0) = data.gps_pos_x;
    z_gps(1) = data.gps_pos_y;
    z_gps(2) = data.gps_vel_x;
    z_gps(3) = data.gps_vel_y;
    
    // Adaptive noise based on GPS accuracy
    Matrix<GPS_MEAS_SIZE, GPS_MEAS_SIZE> R_adaptive = R_gps;
    float noise_factor = max(1.0f, data.gps_accuracy / 5.0f);
    R_adaptive(0,0) = R_gps(0,0) * noise_factor;
    R_adaptive(1,1) = R_gps(1,1) * noise_factor;
    R_adaptive(2,2) = R_gps(2,2) * noise_factor;
    R_adaptive(3,3) = R_gps(3,3) * noise_factor;
    
    // Kalman update
    performUpdate(z_gps, H_gps, R_adaptive);
  }
  
  void updateIMU(const SensorData& data) {
    // data.accel_x and data.accel_y are now body-frame, bias-corrected,
    // gravity-compensated, and filtered linear accelerations (in m/s^2).
    // data.heading, data.pitch, data.roll are current orientation estimates (in radians).

    float cos_h = cos(data.heading);
    float sin_h = sin(data.heading);
    // Pitch and Roll are less critical for 2D planar motion if IMU is mostly flat,
    // but included for completeness if there's tilt.
    // float cos_p = cos(data.pitch);
    // float sin_p = sin(data.pitch);
    // float cos_r = cos(data.roll);
    // float sin_r = sin(data.roll);

    // Simplified rotation for 2D: only heading matters for transforming body X/Y to world X/Y
    // Assumes IMU's X-axis is car's forward, Y-axis is car's left.
    // World X is East, World Y is North (or some consistent reference).
    // If GPS velocities (velE, velN) are used, world X should align with East, world Y with North.
    
    float ax_body_linear = data.accel_x;
    float ay_body_linear = data.accel_y;

    // Transform body-frame linear accelerations to world frame using heading
    // accel_world_x = ax_body_linear * cos_h - ay_body_linear * sin_h;
    // accel_world_y = ax_body_linear * sin_h + ay_body_linear * cos_h;
    // This common rotation assumes:
    //  - Heading=0 means body X aligns with world X.
    //  - Positive heading is counter-clockwise rotation from world X.
    //  - Body Y is 90deg CCW from body X.
    //  - World Y is 90deg CCW from world X.
    // Let's align with typical GPS NED (North-East-Down) where X might be North, Y East.
    // And car forward (body X) has heading relative to North (world Y):
    // accel_world_x (East)  = ax_body_linear * sin_h + ay_body_linear * cos_h;
    // accel_world_y (North) = ax_body_linear * cos_h - ay_body_linear * sin_h;
    // Let's stick to a common convention: World X = East, World Y = North.
    // Heading is angle from North, positive clockwise (typical for navigation).
    // If so, cos_h = cos(heading_from_North_CW), sin_h = sin(heading_from_North_CW)
    // World_X_East  = Body_X_fwd * sin_h + Body_Y_left * cos_h
    // World_Y_North = Body_X_fwd * cos_h - Body_Y_left * sin_h
    // The current sensorData.heading is from gyro integration, likely relative to startup.
    // For now, let's use the simpler rotation assuming heading is angle from world X:
    float accel_world_x = ax_body_linear * cos_h - ay_body_linear * sin_h;
    float accel_world_y = ax_body_linear * sin_h + ay_body_linear * cos_h;


    Matrix<IMU_MEAS_SIZE, 1> z_imu;
    z_imu(0) = accel_world_x;
    z_imu(1) = accel_world_y;
    
    // R_imu should reflect the uncertainty of the world-frame accelerations
    // This depends on the quality of orientation estimation and IMU noise.
    performUpdate(z_imu, H_imu, R_imu);
  }
  
  FilterOutput getOutput() {
    FilterOutput output;
    
    output.position_x = x(0);
    output.position_y = x(1);
    output.velocity_x = x(2);
    output.velocity_y = x(3);
    output.acceleration_x = x(4);
    output.acceleration_y = x(5);
    
    output.speed_ms = sqrt(x(2)*x(2) + x(3)*x(3));
    output.speed_kmh = output.speed_ms * 3.6f;
    output.heading = atan2(x(3), x(2));
    
    // Calculate confidence from trace of velocity covariance
    output.confidence = 1.0f / (1.0f + P(2,2) + P(3,3));
    output.valid = (output.confidence > 0.1f);
    
    return output;
  }  void adaptBiases(const SensorData& data, bool gps_recently_valid) {
    // Calculate current velocity magnitude from Kalman filter state (world frame)
    float velocity_magnitude_world = sqrt(x(2)*x(2) + x(3)*x(3));
    
    // data.accel_x, data.accel_y are body-frame linear accelerations (m/s^2)
    // data.gyro_z is bias-corrected gyro_z rate (rad/s)
    float body_linear_accel_magnitude = sqrt(data.accel_x*data.accel_x + data.accel_y*data.accel_y);

    // Adapt biases when nearly stationary (low world velocity) and low body linear acceleration
    // and GPS is considered somewhat reliable (e.g. recent valid fix)
    bool can_adapt = data.gps_valid || gps_recently_valid;

    if (can_adapt && velocity_magnitude_world < 0.2f && body_linear_accel_magnitude < 0.3f) { // Stricter stationary thresholds
      float adaptation_rate_accel = 0.002f; // Slower adaptation
      float adaptation_rate_gyro = 0.002f;  // Slower adaptation

      // If stationary, true body linear acceleration (data.accel_x, data.accel_y) should be 0.
      // Any non-zero value is an error that the bias should correct.
      // accel_bias_x is (old_bias). We want new_bias = old_bias + learning * error
      // error is (measured_linear_accel - target_linear_accel_which_is_0) = data.accel_x
      accel_bias_x += adaptation_rate_accel * data.accel_x; 
      accel_bias_y += adaptation_rate_accel * data.accel_y;

      // If stationary and not rotating, true gyro_z rate (data.gyro_z, which is raw_gyro - current_bias) should be 0.
      // Any non-zero value indicates the current gyro_bias_z is off.
      // error is (measured_corrected_gyro_rate - target_rate_which_is_0) = data.gyro_z
      gyro_bias_z += adaptation_rate_gyro * data.gyro_z;
    }
  }
    void resetVelocity() {
    // Fix velocity calculation and enhance reset logic
    float vel_mag = sqrt(x(2)*x(2) + x(3)*x(3)); // Proper magnitude calculation
    float current_speed_kmh = vel_mag * 3.6f;
    
    if (current_speed_kmh > 5.0f) {
      x(2) = 0; // velocity_x
      x(3) = 0; // velocity_y
      
      // Preserve 50% of acceleration (better than 80% during resets)
      x(4) *= 0.5f;
      x(5) *= 0.5f;
      
      // Increase uncertainty
      P(2,2) = 25;
      P(3,3) = 25;
    }
  }

private:  
  template<int MeasSize>
  void performUpdate(const Matrix<MeasSize, 1>& z, 
                    const Matrix<MeasSize, STATE_SIZE>& H,
                    const Matrix<MeasSize, MeasSize>& R_in) {
    // Innovation
    Matrix<MeasSize, 1> y = z - (H * x);
    
    // Innovation covariance
    Matrix<MeasSize, MeasSize> S_matrix = H * P * (~H) + R_in; // Renamed to avoid confusion
    
    // Kalman gain - Invert S_matrix (the copy)
    // Create a copy for inversion, as Invert likely works in-place
    Matrix<MeasSize, MeasSize> S_inv = S_matrix; 
    bool inversion_successful = Invert(S_inv); // Invert S_inv in-place

    if (!inversion_successful) {
      // Handle the case where inversion fails, e.g., log an error or skip the update
      // For now, we can print a message and return to avoid issues with a non-inverted matrix
      if (Serial) { // Check if Serial is available (it should be in Arduino context)
        Serial.println("Error: Matrix S inversion failed in performUpdate!");
      }
      return; 
    }
    
    Matrix<STATE_SIZE, MeasSize> K = P * (~H) * S_inv; // Use the inverted matrix S_inv
    
    // Update state
    x = x + K * y;
    
    // Update covariance
    Matrix<STATE_SIZE, STATE_SIZE> I;
    I.Fill(0);
    for(int i = 0; i < STATE_SIZE; i++) I(i,i) = 1.0f; // Create identity matrix
    P = (I - K * H) * P;
  }
};

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
LSM6DS3 imu(I2C_MODE, 0x6A); // Use 0x6A or 0x6B depending on SA0 pin
AdvancedKalmanFilter kf;
UbxParser ubxParser;
GpsData gpsData;
WebServer server(80);

// --- Global State Flags ---
bool imuConnected = false;
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
AdvancedKalmanFilter::SensorData sensorData;
AdvancedKalmanFilter::FilterOutput filterOutput;

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
    
    // Convert GPS coordinates to local position
    double lat_deg = gpsData.lat * 1e-7;
    double lon_deg = gpsData.lon * 1e-7;
    gpsToLocal(lat_deg, lon_deg, sensorData.gps_pos_x, sensorData.gps_pos_y);
    
    // Convert GPS velocities from mm/s to m/s
    sensorData.gps_vel_x = (float)gpsData.velE / 1000.0f; // East velocity -> X
    sensorData.gps_vel_y = (float)gpsData.velN / 1000.0f; // North velocity -> Y
    sensorData.gps_vel_n = (float)gpsData.velN / 1000.0f; 
    sensorData.gps_vel_e = (float)gpsData.velE / 1000.0f;
    sensorData.gps_vel_d = (float)gpsData.velD / 1000.0f;
    sensorData.gps_gnd_speed = (float)gpsData.gSpeed / 1000.0f;
    sensorData.gps_sAcc = (float)gpsData.sAcc / 1000.0f;
    sensorData.gps_hAcc = (float)gpsData.hAcc / 1000.0f;
    sensorData.gps_accuracy = sensorData.gps_hAcc; // Use hAcc for accuracy

  } else {
    sensorData.gps_valid = false;
    sensorData.gps_pos_x = 0.0f;
    sensorData.gps_pos_y = 0.0f;
    sensorData.gps_vel_x = 0.0f;
    sensorData.gps_vel_y = 0.0f;
    sensorData.gps_vel_n = 0.0f;
    sensorData.gps_vel_e = 0.0f;
    sensorData.gps_vel_d = 0.0f;
    sensorData.gps_gnd_speed = 0.0f;
    sensorData.gps_sAcc = 100.0f; // High uncertainty
    sensorData.gps_hAcc = 100.0f;
    sensorData.gps_accuracy = 100.0f;
  }
}

void IRAM_ATTR processGpsChar(uint8_t c) {
  if (parseUbxByte(c)) {
    if (ubxParser.msgClass == UBX_CLASS_NAV && ubxParser.msgId == UBX_MSG_NAV_PVT) {
      processUbxNavPvt();
    }
  }
}

// --- IMU Functions ---
void IRAM_ATTR readIMU() {
  if (!imuConnected) {
    sensorData.accel_x = 0; sensorData.accel_y = 0; sensorData.accel_z = 0;
    sensorData.gyro_x = 0; sensorData.gyro_y = 0; sensorData.gyro_z = 0;
    sensorData.heading = 0; sensorData.pitch = 0; sensorData.roll = 0;
    sensorData.timestamp = micros();
    return;
  }

  // 1. Read raw IMU data (accelerometer in g, gyro in dps)
  float raw_ax_g = imu.readFloatAccelX();
  float raw_ay_g = imu.readFloatAccelY();
  float raw_az_g = imu.readFloatAccelZ();
  
  float raw_gx_dps = imu.readFloatGyroX();
  float raw_gy_dps = imu.readFloatGyroY();
  float raw_gz_dps = imu.readFloatGyroZ();
  
  // 2. Optional: Temperature Compensation (basic example)
  // float temp = imu.readTempC();
  // float temp_factor_accel = 1.0f; // + 0.005f * (25.0f - temp); // Example: 0.5%/°C
  // float temp_factor_gyro = 1.0f;  // + 0.002f * (25.0f - temp);  // Example: 0.2%/°C
  // raw_ax_g *= temp_factor_accel;
  // raw_ay_g *= temp_factor_accel;
  // raw_az_g *= temp_factor_accel;
  // raw_gz_dps *= temp_factor_gyro;

  // 3. Convert raw accelerometer data to m/s²
  float accel_x_m_s2_raw = raw_ax_g * 9.81f;
  float accel_y_m_s2_raw = raw_ay_g * 9.81f;
  float accel_z_m_s2_raw = raw_az_g * 9.81f;

  // 4. Estimate Pitch and Roll from raw (but converted to m/s^2) accelerometer data
  sensorData.pitch = atan2(-accel_x_m_s2_raw, sqrt(accel_y_m_s2_raw * accel_y_m_s2_raw + accel_z_m_s2_raw * accel_z_m_s2_raw));
  sensorData.roll  = atan2(accel_y_m_s2_raw, accel_z_m_s2_raw); // Common roll calculation

  // 5. Gyro processing: Convert to rad/s and subtract current KF gyro bias
  float gyro_z_rad_s_corrected = (raw_gz_dps * DEG_TO_RAD) - kf.getGyroBiasZ();
  sensorData.gyro_z = gyro_z_rad_s_corrected; // Store bias-corrected rate for KF adaptBiases

  // 6. Update heading (integrate bias-corrected gyro_z)
  static unsigned long last_imu_time_micros = 0;
  unsigned long current_time_micros = micros();
  if (last_imu_time_micros > 0) {
    float dt_imu_sec = (current_time_micros - last_imu_time_micros) / 1000000.0f;
    if (dt_imu_sec < 0) dt_imu_sec = 0; // Safety for rollover or init
    sensorData.heading += gyro_z_rad_s_corrected * dt_imu_sec;
    // Normalize heading to +/- PI
    while (sensorData.heading > PI) sensorData.heading -= 2.0f * PI;
    while (sensorData.heading < -PI) sensorData.heading += 2.0f * PI;
  }
  last_imu_time_micros = current_time_micros;
  sensorData.timestamp = current_time_micros;

  // 7. Calculate Body-Frame Linear Accelerations:
  float body_ax_bias_removed = accel_x_m_s2_raw - kf.getAccelBiasX();
  float body_ay_bias_removed = accel_y_m_s2_raw - kf.getAccelBiasY();

  const float g = 9.81f;
  float ax_body_linear = body_ax_bias_removed - g * sin(sensorData.pitch);
  float ay_body_linear = body_ay_bias_removed + g * sin(sensorData.roll) * cos(sensorData.pitch); 
  // Note: The sign for ay_body_linear depends on axis convention and roll definition.
  // This assumes positive roll means right side down, and positive ay is to the left.

  // 8. Apply adaptive low-pass filter to body-frame linear accelerations
  static float filtered_ax_body = ax_body_linear; // Initialize with first value
  static float filtered_ay_body = ay_body_linear; // Initialize with first value
  
  float current_body_accel_mag = sqrt(ax_body_linear * ax_body_linear + ay_body_linear * ay_body_linear);
  
  float alpha = 0.5f; // Base alpha
  if (current_body_accel_mag > 5.0f) { // Adjust threshold (5m/s^2 ~0.5g)
    alpha = 0.7f; 
  } else if (current_body_accel_mag < 1.0f) { // Near stationary or smooth motion
    alpha = 0.2f; 
  }
  
  sensorData.accel_x = alpha * ax_body_linear + (1.0f - alpha) * filtered_ax_body;
  sensorData.accel_y = alpha * ay_body_linear + (1.0f - alpha) * filtered_ay_body;
  
  filtered_ax_body = sensorData.accel_x;
  filtered_ay_body = sensorData.accel_y;
  
  // sensorData.accel_z is not directly used by the 2D KF for linear acceleration input.
  // Store the raw (temp-compensated, m/s^2) value for reference or future use.
  sensorData.accel_z = accel_z_m_s2_raw; 
  
  // Store other gyro axes if needed (bias corrected)
  sensorData.gyro_x = (raw_gx_dps * DEG_TO_RAD); // - kf.getGyroBiasX() if you add X bias
  sensorData.gyro_y = (raw_gy_dps * DEG_TO_RAD); // - kf.getGyroBiasY() if you add Y bias
}

// --- Performance Measurement with Kalman Filter ---
void IRAM_ATTR handlePerformanceMeasurement() {
  if (!filterOutput.valid) return;
  
  // Enhanced trigger system using both GPS and accelerometer
  bool gps_trigger = sensorData.gps_valid && (filterOutput.speed_kmh >= START_TRIGGER_KMH);
  bool accel_trigger = (filterOutput.acceleration_x >= ACCEL_START_THRESHOLD) && (filterOutput.confidence > 0.7f);
  
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

// --- Web Interface (simplified) ---
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-S3 Performance Tracker</title>
    <meta name='viewport' content='width=device-width,initial-scale=1'>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f4f4f4; color: #333; }
        .container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        h1 { color: #0056b3; }
        .data-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 20px; }
        .data-item { background-color: #e9ecef; padding: 15px; border-radius: 4px; }
        .data-item strong { color: #0056b3; }
        button { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; margin-right: 10px; }
        button:hover { background-color: #0056b3; }
        input[type="number"] { padding: 8px; margin-right: 10px; border: 1px solid #ccc; border-radius: 4px; }
        .controls { margin-top: 20px; margin-bottom: 20px; }
        #log { margin-top: 20px; background-color: #333; color: #fff; padding: 10px; border-radius: 4px; height: 150px; overflow-y: scroll; font-family: monospace; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Advanced Performance Tracker</h1>
        
        <div class="controls">
            <button onclick="fetch('/arm')">ARM</button>
            <button onclick="fetch('/reset')">RESET</button>
            <input type="number" id="targetSpeedInput" value="100" step="1">
            <button onclick="setTargetSpeed()">Set Target Speed (km/h)</button>
        </div>

        <div class="data-grid">
            <div class="data-item"><strong>Status:</strong> <span id="status">IDLE</span></div>
            <div class="data-item"><strong>Current Speed:</strong> <span id="speed">0.00</span> km/h</div>
            <div class="data-item"><strong>Elapsed Time:</strong> <span id="time">0.000</span> s</div>
            <div class="data-item"><strong>Target Speed:</strong> <span id="targetSpeedDisplay">100.0</span> km/h</div>
            <div class="data-item"><strong>Filter Confidence:</strong> <span id="conf">0.00</span></div>
            <div class="data-item"><strong>Accel X:</strong> <span id="ax">0.00</span> m/s²</div>
            <div class="data-item"><strong>Accel Y:</strong> <span id="ay">0.00</span> m/s²</div>
            <div class="data-item"><strong>GPS Fix Type:</strong> <span id="gpsFix">N/A</span></div>
            <div class="data-item"><strong>GPS Sats:</strong> <span id="gpsSats">0</span></div>
            <div class="data-item"><strong>GPS Lat:</strong> <span id="gpsLat">0.0</span></div>
            <div class="data-item"><strong>GPS Lon:</strong> <span id="gpsLon">0.0</span></div>
            <div class="data-item"><strong>GPS hAcc:</strong> <span id="gpsHAcc">0</span> mm</div>
            <div class="data-item"><strong>GPS gSpeed:</strong> <span id="gpsGSpeed">0</span> mm/s</div>            <div class="data-item"><strong>GPS velN:</strong> <span id="gpsVelN">0</span> mm/s</div>            <div class="data-item"><strong>GPS velE:</strong> <span id="gpsVelE">0</span> mm/s</div>
            <div class="data-item"><strong>GPS Timeout:</strong> <span id="gpsTimeout">No</span> (<span id="gpsTimeoutMs">0</span>ms)</div>
            <div class="data-item"><strong>Raw Accel X:</strong> <span id="rawAx">0.00</span> g</div>
            <div class="data-item"><strong>Raw Accel Y:</strong> <span id="rawAy">0.00</span> g</div>
            <div class="data-item"><strong>Raw Accel Z:</strong> <span id="rawAz">0.00</span> g</div>
            <div class="data-item"><strong>IMU Heading:</strong> <span id="imuHeading">0.00</span>°</div>
        </div>
        <div id="log"></div>
    </div>

    <script>
        function setTargetSpeed() {
            const speed = document.getElementById('targetSpeedInput').value;
            fetch('/settarget?speed=' + speed)
                .then(response => response.text())
                .then(text => { 
                    console.log(text); 
                    document.getElementById('targetSpeedDisplay').textContent = parseFloat(speed).toFixed(1);
                    addToLog('Target speed set to ' + speed + ' km/h');
                });
        }

        function addToLog(message) {
            const logDiv = document.getElementById('log');
            const time = new Date().toLocaleTimeString();
            logDiv.innerHTML += '[' + time + '] ' + message + '<br>';
            logDiv.scrollTop = logDiv.scrollHeight;
        }

        setInterval(() => {
            fetch('/data')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('speed').textContent = d.s.toFixed(2);
                    document.getElementById('time').textContent = d.t.toFixed(3);
                    document.getElementById('status').textContent = d.st;
                    document.getElementById('conf').textContent = d.c.toFixed(2);
                    document.getElementById('ax').textContent = d.ax.toFixed(2);
                    document.getElementById('ay').textContent = d.ay.toFixed(2);
                    document.getElementById('targetSpeedDisplay').textContent = d.ts.toFixed(1);
                    document.getElementById('targetSpeedInput').value = d.ts.toFixed(1); // Keep input in sync

                    document.getElementById('gpsFix').textContent = d.gps_fix;
                    document.getElementById('gpsSats').textContent = d.gps_sats;
                    document.getElementById('gpsLat').textContent = d.gps_lat.toFixed(7);
                    document.getElementById('gpsLon').textContent = d.gps_lon.toFixed(7);
                    document.getElementById('gpsHAcc').textContent = d.gps_h_acc;                    document.getElementById('gpsGSpeed').textContent = d.gps_g_speed;
                    document.getElementById('gpsVelN').textContent = d.gps_vel_n;
                    document.getElementById('gpsVelE').textContent = d.gps_vel_e;                    document.getElementById('rawAx').textContent = d.raw_ax.toFixed(3);
                    document.getElementById('rawAy').textContent = d.raw_ay.toFixed(3);
                    document.getElementById('rawAz').textContent = d.raw_az.toFixed(3);
                    document.getElementById('imuHeading').textContent = (d.imu_heading * 180/3.14159).toFixed(1);
                    
                    // GPS timeout status
                    document.getElementById('gpsTimeout').textContent = d.gps_timeout === "true" ? "YES" : "No";
                    document.getElementById('gpsTimeoutMs').textContent = d.gps_timeout_ms;
                    
                    if(d.st_changed) { // If status changed, log it
                        addToLog("State changed to: " + d.st);
                    }
                    
                    // Log GPS timeout events
                    if(d.gps_timeout === "true" && !window.lastGpsTimeoutLogged) {
                        addToLog("GPS timeout detected - velocity reset active");
                        window.lastGpsTimeoutLogged = true;
                    } else if(d.gps_timeout === "false" && window.lastGpsTimeoutLogged) {
                        addToLog("GPS signal recovered");
                        window.lastGpsTimeoutLogged = false;
                    }
                })
                .catch(e => {
                    console.error("Error fetching data:", e);
                    addToLog("Error fetching data: " + e);                });
        }, 100); // Reduced to 100ms for faster UI updates
        
        addToLog("Page loaded. Fetching data...");
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
  json += ",\"ax\":" + String(filterOutput.acceleration_x, 2);
  json += ",\"ay\":" + String(filterOutput.acceleration_y, 2);
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
  json += ",\"raw_ax\":" + String(imu.readFloatAccelX(), 3);
  json += ",\"raw_ay\":" + String(imu.readFloatAccelY(), 3);
  json += ",\"raw_az\":" + String(imu.readFloatAccelZ(), 3);
  json += ",\"imu_heading\":" + String(sensorData.heading, 3);
    // Kalman filter state debugging
  json += ",\"filter_valid\":" + String(filterOutput.valid ? "true" : "false");
  json += ",\"filter_vel_x\":" + String(filterOutput.velocity_x, 2);
  json += ",\"filter_vel_y\":" + String(filterOutput.velocity_y, 2);
  json += ",\"gps_valid\":" + String(sensorData.gps_valid ? "true" : "false");
  
  // GPS timeout status
  static unsigned long lastValidGPS = 0;
  if (sensorData.gps_valid) lastValidGPS = micros();
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
  
  // Initialize I2C for IMU
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000); // Fast I2C
  
  // Configure IMU settings before initialization
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 8;      // CHANGED from 16 to 8 (for ±8g)
  imu.settings.accelSampleRate = 833; // Options: 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660 Hz
  imu.settings.accelBandWidth = 400;  // Options: 50, 100, 200, 400 Hz (or ODR/2, ODR/4, etc.)
  
  imu.settings.gyroEnabled = 1;
  imu.settings.gyroRange = 2000;    // Options: 125, 250, 500, 1000, 2000 dps
  imu.settings.gyroSampleRate = 833;  // Match accel rate if possible, or suitable rate
  // imu.settings.gyroBandWidth = ODR based, check datasheet if specific BW needed
  
  imu.settings.tempEnabled = 1; // Enable temperature sensor
  
  // Initialize IMU with configured settings
  if (imu.begin() != 0) {
    Serial.println("LSM6DS3 initialization failed!");
    imuConnected = false;
  } else {
    Serial.println("LSM6DS3 initialized successfully!");
    imuConnected = true;
    Serial.print("Accelerometer range: ±"); Serial.print(imu.settings.accelRange); Serial.println("g");
    Serial.print("Accelerometer rate: "); Serial.print(imu.settings.accelSampleRate); Serial.println(" Hz");
    Serial.print("Gyroscope range: ±"); Serial.print(imu.settings.gyroRange); Serial.println(" deg/s");
    Serial.print("Gyroscope rate: "); Serial.print(imu.settings.gyroSampleRate); Serial.println(" Hz");
  }
  
  // Add calibration routine
  Serial.println("Calibrating IMU...");
  float ax_sum_g = 0, ay_sum_g = 0, gz_sum_dps = 0; // Store sums in raw units first
  const int calibration_samples = 500;

  if (imuConnected) { // Only calibrate if IMU is connected
    Serial.println("Place the IMU on a flat, stable surface and do not move it during calibration.");
    delay(2000); // Give time to stabilize
    for(int i = 0; i < calibration_samples; i++) {
      ax_sum_g += imu.readFloatAccelX();
      ay_sum_g += imu.readFloatAccelY();
      // Assuming Z-axis is vertical, accel Z should be ~1g or -1g. We don't calibrate Z accel bias this way.
      gz_sum_dps += imu.readFloatGyroZ();
      delay(5); // Small delay between readings
    }
    // Convert sums to averages and then to standard units for biases
    kf.getAccelBiasX() = (ax_sum_g / calibration_samples) * 9.81f; // g to m/s^2
    kf.getAccelBiasY() = (ay_sum_g / calibration_samples) * 9.81f; // g to m/s^2
    kf.getGyroBiasZ() = (gz_sum_dps / calibration_samples) * DEG_TO_RAD; // dps to rad/s

    Serial.printf("Calibration complete. Accel Biases: X=%.4f m/s^2, Y=%.4f m/s^2\\n",
                  kf.getAccelBiasX(), kf.getAccelBiasY());
    Serial.printf("Gyro Bias Z: %.4f rad/s\\n", kf.getGyroBiasZ());
  } else {
    Serial.println("IMU not connected. Skipping calibration. Biases will be zero.");
    // Biases will remain 0.0f as initialized in the KF class
  }

  // Initialize Kalman filter
  kf.initialize();
  
  // Setup interrupts
  pinMode(IMU_INT1_PIN, INPUT);
  pinMode(IMU_INT2_PIN, INPUT);

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

  static unsigned long lastKalmanUpdate = 0;
  static unsigned long lastIMURead = 0;
  
  unsigned long currentTime = micros();
  
  // High-frequency IMU reading (target ~1kHz, actual depends on readIMU execution time)
  if (imuConnected && (currentTime - lastIMURead >= 1000)) { // Only read if connected
    readIMU();
    lastIMURead = currentTime;
  }
  
  // Process GPS data
  while (GPS_SERIAL.available()) {
    processGpsChar(GPS_SERIAL.read());
  }
  
  // Kalman filter update logic
  const unsigned long GPS_TIMEOUT_US = 2000000; // 2 seconds GPS timeout
  
  // Track last valid GPS time (moved lastValidGPS to global)
  if (sensorData.gps_valid) {
    lastValidGPS = currentTime;
  }
  
  // Check for GPS timeout and reset velocity if needed
  if (lastValidGPS > 0 && (currentTime - lastValidGPS) > GPS_TIMEOUT_US) {
    static unsigned long lastResetTime = 0;
    // Reset velocity every 500ms during GPS timeout to prevent drift
    if ((currentTime - lastResetTime) > 500000) {
      kf.resetVelocity();
      lastResetTime = currentTime;
      // Serial.println("GPS Timeout - KF Velocity Reset"); // Optional debug
    }
  }
  
  static unsigned long lastGPSUpdateKF = 0; // Renamed to avoid conflict with local var in handleData

  // GPS-triggered update (every 45ms when GPS data is available)
  if (sensorData.gps_valid && (currentTime - lastGPSUpdateKF >= 45000)) {
    float dt_sec = (currentTime - lastKalmanUpdate) / 1000000.0f;
    if (dt_sec <= 0.0001f) dt_sec = 0.001f; // Prevent zero/negative/too small dt
    lastKalmanUpdate = currentTime;
    lastGPSUpdateKF = currentTime;
      kf.predict(dt_sec);
    kf.updateGPS(sensorData); // GPS update first
    if (imuConnected) {
        kf.updateIMU(sensorData); // Then IMU update
        bool gps_recently_valid = (lastValidGPS > 0) && ((currentTime - lastValidGPS) < 1000000); // 1 second
        kf.adaptBiases(sensorData, gps_recently_valid);
    }
    
    sensorData.gps_valid = false; // Reset flag after use
    filterOutput = kf.getOutput();
    handlePerformanceMeasurement();

  }  // IMU-only update (e.g., every 5ms when no recent GPS, for responsive acceleration)
  else if (imuConnected && (currentTime - lastKalmanUpdate >= 5000)) { // Approx 200Hz
    float dt_sec = (currentTime - lastKalmanUpdate) / 1000000.0f;
    if (dt_sec <= 0.0001f) dt_sec = 0.001f; // Prevent zero/negative/too small dt
    lastKalmanUpdate = currentTime;
      kf.predict(dt_sec);
    kf.updateIMU(sensorData);
    bool gps_recently_valid = (lastValidGPS > 0) && ((currentTime - lastValidGPS) < 1000000); // 1 second
    kf.adaptBiases(sensorData, gps_recently_valid);
    
    filterOutput = kf.getOutput();
    handlePerformanceMeasurement();
  }
  
  server.handleClient();
  if (currentTime % 10000 == 0) yield(); // Yield every 10ms to prevent watchdog
}

// --- AdvancedKalmanFilter class and UBX parser code (unchanged) ---