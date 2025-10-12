<!-- Repository-specific instructions for AI coding assistants -->
# Copilot instructions — gnss-speed-s3

Short, focused guidance for code edits and feature work in this PlatformIO ESP32-S3 project.

Key ideas
- This is a single-microcontroller (ESP32-S3) firmware project using PlatformIO + Arduino framework.
- Main application logic lives in `src/main.cpp` — it contains the GPS UBX parser, LSM6DS3 IMU handling, an AdvancedKalmanFilter class (uses BasicLinearAlgebra), and a small WebServer-based UI.
- The build environment is defined in `platformio.ini` under the environment `[env:esp32-s3-devkitc-1]`.

What to look at first
- `src/main.cpp` — the canonical entrypoint. Major responsibilities:
  - GPS UBX parsing state machine (functions: `parseUbxByte`, `processUbxNavPvt`, `processGpsChar`).
  - IMU read and preprocessing (`readIMU`) and chip setup (SparkFun LSM6DS3).
  - `AdvancedKalmanFilter` class: state vector layout and update/predict flows. Filters use `BasicLinearAlgebra` matrices.
  - Web server endpoints: `/` (UI), `/data` (JSON), `/arm`, `/reset`, `/settarget`.
  - Timing uses `micros()` and `millis()` extensively; many timeouts expressed in microseconds.

Important conventions & patterns
- Units matter: GPS UBX fields are mostly in mm or mm/s — the code converts to meters/meters-per-second. IMU accel is read in g and converted to m/s².
- Coordinate frames: GPS velocities are parsed as N/E/D; code maps East->X and North->Y for local coordinates. `gpsToLocal()` uses a reference lat/lon (set on first fix).
- The Kalman filter state vector is: [pos_x, pos_y, vel_x, vel_y, accel_x, accel_y]. H matrices indicate which measurements map to which state entries.
- Sensor bias handling: IMU biases are estimated/calibrated in `setup()` (simple averaging) and adapted online in `AdvancedKalmanFilter::adaptBiases()`.
- Safety: many sections assume an IMU is present; branches handle `imuConnected` false. Avoid removing those guards.

Build, upload, and debug (PlatformIO / Windows PowerShell)
- Build: in project root run (PowerShell):
  pio run
- Build & upload to the board (uses env `esp32-s3-devkitc-1`):
  pio run -e esp32-s3-devkitc-1 -t upload
- Serial monitor (default 115200 as set in `Serial.begin(115200)`):
  pio device monitor --baud 115200
- If editing `platformio.ini`, note important libs:
  - sparkfun/SparkFun LSM6DS3 Breakout (IMU)
  - tomstewart89/BasicLinearAlgebra (matrix math used in KF)

Runtime + debugging tips
- Web UI: the device creates an AP named `ESP32-Performance-Tracker`. Visit `http://192.168.4.1/` when connected to the AP. The UI polls `/data` every 100ms.
- Useful Serial outputs:
  - Boot prints after IMU init and calibration. Serial prints calibration results and IMU settings.
  - Use `Serial.println()` liberally when adding sensor/timeout/debug logic; the project already prints from several failure branches.
- To reproduce GPS parsing problems, capture raw UBX traffic from `Serial2` pins (GPIO 44 RX, 43 TX). Don't change those pin defines without confirming hardware wiring.

Common edits & examples
- Add a new Kalman measurement: inspect `AdvancedKalmanFilter::performUpdate` signature and provide a measurement vector `z` with corresponding `H` row mapping.
- To tune GPS influence: update `R_gps` matrix in `AdvancedKalmanFilter::initialize()` or adaptively in `updateGPS()` — the code already scales with `gps_accuracy`.
- To change IMU pinout or I2C speed, edit the constants near top of `main.cpp` (`IMU_SDA_PIN`, `IMU_SCL_PIN`) and the `Wire.setClock()` call.

Patterns to preserve
- Keep timing in microseconds (`micros()`) for IMU/GPS loops. Many thresholds are expressed in microseconds; converting half of the code to milliseconds will cause subtle bugs.
- Maintain measurement unit conversions (mm->m, g->m/s², dps->rad/s). Tests or code reviews should focus on unit consistency.

Where tests and CI are missing
- There are no automated unit tests or CI configuration in this repo. For safe refactors, run `pio run` locally and use `pio device monitor` to verify behavior on hardware.

Useful search queries for maintainers
- Find Kalman references: `grep -n "AdvancedKalmanFilter" -R src`
- Find GPS/UBX code: `grep -n "UBX" -R src`
- List I/O pin definitions: search for `GPS_RX_PIN`, `GPS_TX_PIN`, `IMU_SDA_PIN`, `IMU_SCL_PIN`.

If you are an AI agent making edits
- Don't change hardware pin defines or `platformio.ini` environment names without confirming intent.
- Preserve the UBX parser structure — it's delicate and relies on stateful byte-by-byte parsing.
- When adding new library dependencies, update `platformio.ini` `lib_deps` and prefer pinned semver as used today.

Questions for the maintainer (if unclear)
- Which coordinate frame do you prefer for output (East/North vs North/East)? The code currently maps East->X, North->Y.
- Is there an intended OTA or serial-flash workflow (currently uploads via PlatformIO only)?

If anything in this file is unclear or missing, tell me what you want clarified and I will refine these instructions.
