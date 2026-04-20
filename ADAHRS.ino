#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/* ================= WIFI CONFIG ================= */
const char* ssid     = "RUDRA_14";
const char* password = "Rudra@wifi123";

/* ================= WEB SERVER ================== */
WebServer server(80);

/* ================= I2C PINS (ESP32) ============ */
#define I2C_SDA 21
#define I2C_SCL 22

/* ================= SENSORS ===================== */
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;   // BMP180/BMP085

/* ================= FUSION STATE ================ */
float roll  = 0.0f;    // rad
float pitch = 0.0f;    // rad
float yaw   = 0.0f;    // rad (relative yaw, will drift)

float z_est = 0.0f;    // fused altitude (m, relative)
float v_est = 0.0f;    // fused vertical speed (m/s)

// Kalman covariance
float P00 = 0.0f, P01 = 0.0f;
float P10 = 0.0f, P11 = 0.0f;

// Noise
float Q_pos = 0.01f;
float Q_vel = 0.15f;
float R_alt = 0.3f;

const float G = 9.81f;         // m/s^2
unsigned long lastMicros = 0;  // for dt

// Baro baseline
bool  baroCalibrated = false;
float z_baro0      = 0.0f;     // baseline altitude
float z_baro_filt  = 0.0f;     // filtered relative altitude
const float baroAlpha = 0.92f;

// Gyro bias
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Accel low-pass
float ax_f = 0.0f;
float ay_f = 0.0f;
float az_f = 0.0f;
const float accelAlpha = 0.4f;

// Sensor update timing
unsigned long lastSensorUpdate = 0;
// Run fusion at 50 ms (20 Hz)
const unsigned long SENSOR_INTERVAL_MS = 50;

// Baro throttling (~6–7 Hz)
float lastBaroAlt = 0.0f;
int   baroDiv     = 0;
const int BARO_DIV_MAX = 2;  // 20 Hz / (2+1) ≈ ~6–7 Hz baro updates

// Serial print timing (1 Hz)
unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_INTERVAL_MS = 1000;

/* ============ STATIC HTML PAGE ================= */

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32 Flight Telemetry</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f5f5f5; }
    .card {
      max-width: 480px;
      margin: 30px auto;
      background: #ffffff;
      padding: 20px;
      border-radius: 12px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.1);
    }
    h2 { margin-top: 0; }
    .label { color: #555; }
    .value { font-size: 24px; font-weight: bold; }
  </style>
</head>
<body>
  <div class="card">
    <h2>ESP32 Flight Telemetry</h2>
    <p class="label">Fused Altitude (m): <span id="alt" class="value">--</span></p>
    <p class="label">Vertical Speed (m/s): <span id="vz" class="value">--</span></p>
    <hr>
    <p class="label">Roll (°): <span id="roll" class="value">--</span></p>
    <p class="label">Pitch (°): <span id="pitch" class="value">--</span></p>
    <p class="label">Yaw (°): <span id="yaw" class="value">--</span></p>
    <hr>
    <p class="label">Last update: <span id="ts">--</span></p>
  </div>

  <script>
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(d => {
          document.getElementById('alt').innerText   = d.alt.toFixed(2);
          document.getElementById('vz').innerText    = d.vz.toFixed(2);
          document.getElementById('roll').innerText  = d.roll.toFixed(1);
          document.getElementById('pitch').innerText = d.pitch.toFixed(1);
          document.getElementById('yaw').innerText   = d.yaw.toFixed(1);
          document.getElementById('ts').innerText    = new Date().toLocaleTimeString();
        })
        .catch(err => {
          console.log(err);
        });
    }

    // Refresh once per second
    setInterval(updateData, 1000);
    window.onload = updateData;
  </script>
</body>
</html>
)rawliteral";

/* ============ GYRO BIAS CALIBRATION ============ */
void calibrateGyro() {
  Serial.println("Calibrating gyro... keep sensor absolutely still.");
  const int N = 500;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;
    delay(3);
  }
  gyroBiasX /= N;
  gyroBiasY /= N;
  gyroBiasZ /= N;

  Serial.print("Gyro bias X (deg/s): ");
  Serial.println(gyroBiasX * 180.0f / PI, 4);
  Serial.print("Gyro bias Y (deg/s): ");
  Serial.println(gyroBiasY * 180.0f / PI, 4);
  Serial.print("Gyro bias Z (deg/s): ");
  Serial.println(gyroBiasZ * 180.0f / PI, 4);
}

/* ============ KALMAN PREDICT =================== */
void kalmanPredict(float a_z, float dt) {
  z_est += v_est * dt + 0.5f * a_z * dt * dt;
  v_est += a_z * dt;

  float P00_old = P00;
  float P01_old = P01;
  float P10_old = P10;
  float P11_old = P11;

  P00 = P00_old + dt * (P10_old + P01_old) + dt * dt * P11_old + Q_pos;
  P01 = P01_old + dt * P11_old;
  P10 = P10_old + dt * P11_old;
  P11 = P11_old + Q_vel;
}

/* ============ KALMAN UPDATE ==================== */
void kalmanUpdate(float z_meas) {
  float y = z_meas - z_est;      // innovation
  float S = P00 + R_alt;         // innovation covariance
  float K0 = P00 / S;
  float K1 = P10 / S;

  z_est += K0 * y;
  v_est += K1 * y;

  float P00_old = P00;
  float P01_old = P01;

  P00 = (1.0f - K0) * P00_old;
  P01 = (1.0f - K0) * P01_old;
  // P10/P11 approximation is fine for this use
}

/* ============ ORIENTATION & VERTICAL ACCEL ===== */
float computeVerticalAccel(float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float dt) {
  // Low-pass accel
  ax_f = accelAlpha * ax_f + (1.0f - accelAlpha) * ax;
  ay_f = accelAlpha * ay_f + (1.0f - accelAlpha) * ay;
  az_f = accelAlpha * az_f + (1.0f - accelAlpha) * az;

  // Remove gyro bias
  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  // Integrate gyro
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;   // relative yaw (will drift)

  // Accel-based angles
  float acc_roll  = atan2(ay_f, az_f);
  float acc_pitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f));

  // Complementary filter
  const float alpha = 0.985f;
  roll  = alpha * roll  + (1.0f - alpha) * acc_roll;
  pitch = alpha * pitch + (1.0f - alpha) * acc_pitch;

  // Rotate accel to world Z and subtract gravity
  float cph = cos(roll), sph = sin(roll);
  float cth = cos(pitch), sth = sin(pitch);

  float a_z_world = -sth * ax_f + sph * cth * ay_f + cph * cth * az_f;
  float a_z_linear = a_z_world - G;

  // Deadband to avoid integrating pure noise
  const float AZ_DEADBAND = 0.06f; // m/s^2
  if (fabs(a_z_linear) < AZ_DEADBAND) {
    a_z_linear = 0.0f;
  }

  return a_z_linear;
}

/* ============ WEB HANDLERS ===================== */
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  char buf[160];
  snprintf(
    buf, sizeof(buf),
    "{\"alt\":%.4f,\"vz\":%.4f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
    z_est,
    v_est,
    roll  * 180.0f / PI,
    pitch * 180.0f / PI,
    yaw   * 180.0f / PI
  );
  server.send(200, "application/json", buf);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

/* ============ SETUP ============================ */
void setup() {
  Serial.begin(9600);
  delay(200);

  Serial.println();
  Serial.println("ESP32 IMU + BMP180 Altitude Fusion + WebServer");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) delay(10);
  }
  Serial.println("MPU6050 found");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Baro
  if (!bmp.begin()) {
    Serial.println("Failed to find BMP180/BMP085");
    while (1) delay(10);
  }
  Serial.println("BMP180/BMP085 found");

  delay(500);
  calibrateGyro();

  lastMicros = micros();

  // WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // keep latency stable
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  // Web endpoints
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

/* ============ LOOP ============================= */
void loop() {
  unsigned long nowMs = millis();

  // Sensor + fusion update (20 Hz)
  if (nowMs - lastSensorUpdate >= SENSOR_INTERVAL_MS) {
    lastSensorUpdate = nowMs;

    unsigned long now = micros();
    float dt = (now - lastMicros) / 1e6f;
    if (dt <= 0.0f) dt = 1e-3f;
    if (dt > 0.05f) dt = 0.05f;
    lastMicros = now;

    // IMU read
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float a_z = computeVerticalAccel(
      a.acceleration.x,
      a.acceleration.y,
      a.acceleration.z,
      g.gyro.x,
      g.gyro.y,
      g.gyro.z,
      dt
    );

    // Throttled baro read (~6–7 Hz)
    const float SEA_LEVEL_PA = 101325.0f;
    if (++baroDiv > BARO_DIV_MAX) {
      lastBaroAlt = bmp.readAltitude(SEA_LEVEL_PA);
      baroDiv = 0;
    }
    float z_baro = lastBaroAlt;

    if (!baroCalibrated) {
      z_baro0 = z_baro;
      z_baro_filt = 0.0f;
      baroCalibrated = true;
    }

    float z_rel = z_baro - z_baro0;
    z_baro_filt = baroAlpha * z_baro_filt + (1.0f - baroAlpha) * z_rel;

    kalmanPredict(a_z, dt);
    kalmanUpdate(z_baro_filt);
  }

  // Serial debug print once per second
  if (nowMs - lastSerialPrint >= SERIAL_INTERVAL_MS) {
    lastSerialPrint = nowMs;

    Serial.print("Alt(m): ");
    Serial.print(z_est, 2);
    Serial.print(" | Vz(m/s): ");
    Serial.print(v_est, 2);
    Serial.print(" | Roll(deg): ");
    Serial.print(roll * 180.0f / PI, 1);
    Serial.print(" | Pitch(deg): ");
    Serial.print(pitch * 180.0f / PI, 1);
    Serial.print(" | Yaw(deg): ");
    Serial.println(yaw * 180.0f / PI, 1);
  }

  // Serve HTTP clients
  server.handleClient();
}
