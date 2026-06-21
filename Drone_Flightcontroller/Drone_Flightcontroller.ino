#include <Arduino.h>
#include <AlfredoCRSF.h>

#include "mpu9250.h"
#include "MadgwickAHRS.h"

#include <HardwareSerial.h>
#include <ESP32Servo.h>

// only to turn them off
#include <WiFi.h>
#include <BluetoothSerial.h>

// ============================================================
//  PIN & HARDWARE CONFIG
// ============================================================
const int MOTOR_PINS[4] = {13, 12, 14, 27};
// 13 = M1 FL, 12 = M2 BL, 14 = M3 FR, 27 = M4 BR 

// ============================================================
//  OBJECTS
// ============================================================
HardwareSerial crsfSerial(2);
AlfredoCRSF    crsf;
Servo          motors[4];

bfs::Mpu9250 imu;
Madgwick filter;

// ============================================================
//  PID GAINS
// ============================================================
float Kp_roll = 1.2f,  Ki_roll = 0.07f,  Kd_roll = 0.02f;
float Kp_pitch = 1.2f, Ki_pitch = 0.07f, Kd_pitch = 0.02f;
float Kp_yaw = 0.0f,   Ki_yaw = 0.00f,   Kd_yaw = 0.00f;

// ============================================================
//  SAFETY LIMITS
// ============================================================
const float I_LIMIT          = 30.0f;
const int   THROTTLE_ARM     = 1060;
const float GYRO_LPF_ALPHA   = 0.15f;

// Max rates for setpoint scaling
const float MAX_ROLL_DEG     = 35.0f;   // max roll angle (in deg)
const float MAX_PITCH_DEG    = 35.0f;   // max pitch angle (in deg)
const float MAX_YAW_RATE     = 90.0f;   // max yaw rate (in deg/s)

// ============================================================
//  PID STATE
// ============================================================
float integral_roll  = 0, integral_pitch  = 0, integral_yaw  = 0;
uint32_t last_pid_time = 0;

// ============================================================
//  LOOP TIMING
// ============================================================
uint32_t main_loop_last_time = 0;

// ============================================================
//  GYRO CALIBRATION OFFSETS
// ============================================================
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
float roll = 0, pitch = 0, yaw = 0;

// ============================================================
//  FAILSAFE TRACKING: disarm if no valid CRSF packet for this long time (ms)
// ============================================================
const uint32_t FAILSAFE_MS = 1000;
uint32_t last_link_up_time = 0;

// ============================================================
//  RC INPUTS  (globals updated each loop)
// ============================================================
float target_roll  = 0;   // deg   (angle setpoint)
float target_pitch = 0;   // deg   (angle setpoint)
float target_yaw   = 0;   // deg/s (rate  setpoint)
int   throttle     = 1000;
bool  armed        = false;

// ============================================================
//  HELPERS
// ============================================================
// Map a CRSF channel value to ±maxVal (centre = 1500, ends = 1000/2000)
inline float channelToRange(int ch, float maxVal) {
  float t = (ch - 1500) / 500.0f;
  return constrain(t, -1.0f, 1.0f) * maxVal;
}

// Map throttle channel (1000–2000) to microseconds (1000–2000)
inline int channelToThrottle(int ch) {
  return constrain(ch, 1000, 2000);
}

inline float deadband(float val, float threshold) {
  return (fabs(val) < threshold) ? 0.0f : val;
}

// ============================================================
//  CALIBRATION
// ============================================================
void calibrateGyro(int samples = 500) {
  Serial.println("Calibrating gyro — keep drone still...");
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < samples; i++) {
    while (!imu.Read()) {}  // wait for new data
    sx += imu.gyro_x_radps();
    sy += imu.gyro_y_radps();
    sz += imu.gyro_z_radps();
  }
  gyro_bias_x = sx / samples;
  gyro_bias_y = sy / samples;
  gyro_bias_z = sz / samples;
  Serial.printf("Bias X: %.5f  Y: %.5f  Z: %.5f\n",
                 gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

// Noise reduction
  WiFi.mode(WIFI_OFF);
  btStop();

// ----- Gyro and Filter setup -----
  Wire.begin(21, 22);
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("IMU init failed! Check wiring.");
    while (1) {}
  }

  imu.ConfigSrd(4);
  imu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_92HZ);      
  filter.begin(200); 
  filter.setBeta(0.15);
  calibrateGyro();

// After calibrateGyro(), warm up the filter for 2000 iterations
 for (int i = 0; i < 2000; i++) {
    while (!imu.Read()) {}
    filter.updateIMU(
      (imu.gyro_x_radps() - gyro_bias_x) * RAD_TO_DEG,
      (imu.gyro_y_radps() - gyro_bias_y) * RAD_TO_DEG,
      (imu.gyro_z_radps() - gyro_bias_z) * RAD_TO_DEG,
      imu.accel_x_mps2(), imu.accel_y_mps2(), -imu.accel_z_mps2()
    );
  }
  roll_offset  = filter.getRoll();   // captures -1.33°
  pitch_offset = filter.getPitch();  // captures 0.75°
  yaw_offset = filter.getYaw();      // captures 175.13°

  Serial.println("=== Gyro ready ===");

  // ----- CRSF Receiver -----
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, 16, 17);
  if (!crsfSerial) {
    while (1) { Serial.println("[ERROR] CRSF serial communication failed!"); delay(1000); }
  }
  crsf.begin(crsfSerial);
  Serial.println("=== CRSF receiver ready ===");

// ----- ESC /Motors setup /startup -----
  for (int i = 0; i < 4; i++) {
    ESP32PWM::allocateTimer(i);
    motors[i].setPeriodHertz(50);
    motors[i].attach(MOTOR_PINS[i], 1000, 2000);
  }

  // ESC calibration sequence
  Serial.println("=== ESC Cal: sending MAX throttle for 3 s ===");
  for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(2000);
  delay(3000);

  Serial.println("=== ESC Cal: sending MIN throttle — arming ESCs... ===");
  for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(1000);
  delay(5000);
  Serial.println("=== ALL 4 MOTORS ARMED ===");

  last_pid_time        = micros();
  main_loop_last_time  = micros();
  last_link_up_time    = millis();
}

// ============================================================
//  MAIN LOGIC LOOP
// ============================================================
void loop() {

// constant loop rate
  uint32_t main_loop_now = micros();

  // Run at ~1 kHz (every 1 ms)
  if ((main_loop_now - main_loop_last_time) < 1000) return;
  main_loop_last_time = main_loop_now;


  if (imu.Read()) {
    // bolderflight uses _radps() for gyro and _mps2() for accel
    // Madgwick expects: gyro in deg/s, accel in any consistent unit
    filter.updateIMU(
      (imu.gyro_x_radps() - gyro_bias_x) * RAD_TO_DEG, // GYRO VAL X
      (imu.gyro_y_radps() - gyro_bias_y) * RAD_TO_DEG, // GYRO VAL Y
      (imu.gyro_z_radps() - gyro_bias_z) * RAD_TO_DEG, // GYRO VAL Z
      imu.accel_x_mps2(),                              // ACELL VAL X
      imu.accel_y_mps2(),                              // ACELL VAL Y
      -imu.accel_z_mps2());                            // ACELL VAL Z

    roll  = filter.getRoll()  - roll_offset;   // --> should read ~0.00°
    pitch = filter.getPitch() - pitch_offset;  // --> should read ~0.00°
    yaw   = filter.getYaw()   - yaw_offset;    // --> should read ~0.00°
   }

// ----------------------------------------------------------
//  1. RC CHANNEL READING
// ----------------------------------------------------------
  crsf.update();

  // Track last time the link was healthy for failsafe timer
  if (crsf.isLinkUp()) {
    last_link_up_time = millis();
  }

  // Failsafe: disarm if link has been down for FAILSAFE_MS
  bool link_ok = (millis() - last_link_up_time) < FAILSAFE_MS;

  if (!link_ok) {
    armed = false;
  } else {
    armed = (crsf.getChannel(8) > 1500);
  }

  // Map channels
  // CH1 = Roll, CH2 = Pitch, CH3 = Throttle, CH4 = Yaw
  // Roll/Pitch --> angle setpoints (deg)
  // Yaw        --> rate  setpoint  (deg/s)   <-- consistent with gyro units
  // Throttle   --> direct µs passthrough
  target_pitch = deadband(channelToRange(crsf.getChannel(2), MAX_PITCH_DEG), 1.0f);
  target_roll  = deadband(channelToRange(crsf.getChannel(1), MAX_ROLL_DEG),  1.0f);
  target_yaw   = channelToRange(crsf.getChannel(4), MAX_YAW_RATE);
  throttle     = channelToThrottle(crsf.getChannel(3));

  // ----------------------------------------------------------
  //  2. DELTA TIME
  // ----------------------------------------------------------
  uint32_t now = micros();
  float dt = (now - last_pid_time) / 1e6f;
  last_pid_time = now;

  // Sanity-check dt: skip this cycle if unreasonable
  if (dt <= 0.0f || dt > 0.1f){
    return;
  }

  // ----------------------------------------------------------
  //  4. PID — ROLL  (angle controller)
  //     P: angle error  (deg)
  //     I: accumulated angle error
  //     D: gyro rate    (deg/s) — the true derivative of angle,
  //        used directly to avoid differentiating noisy angle estimate
  // ----------------------------------------------------------
  float error_roll  = target_roll - roll;
  integral_roll    += error_roll * dt;
  integral_roll     = constrain(integral_roll, -I_LIMIT, I_LIMIT);
  float out_roll    = (Kp_roll  * error_roll)
                    + (Ki_roll  * integral_roll)
                    - (Kd_roll  * ((imu.gyro_x_radps() - gyro_bias_x) * RAD_TO_DEG));     // minus: damps motion

  // ----------------------------------------------------------
  //  5. PID — PITCH  (angle controller, same structure)
  // ----------------------------------------------------------
  float error_pitch  = target_pitch - pitch;
  integral_pitch    += error_pitch * dt;
  integral_pitch     = constrain(integral_pitch, -I_LIMIT, I_LIMIT);
  float out_pitch    = (Kp_pitch * error_pitch)
                     + (Ki_pitch * integral_pitch)
                     - (Kd_pitch * ((imu.gyro_y_radps() - gyro_bias_y) * RAD_TO_DEG));

  // ----------------------------------------------------------
  //  6. PID — YAW  (rate controller)
  //     target_yaw is deg/s; gyro_yaw_f is deg/s
  //     No D term needed for a basic yaw rate controller.
  // ----------------------------------------------------------
  float gyro_z_dps = -(imu.gyro_z_radps() - gyro_bias_z) * RAD_TO_DEG;
  float error_yaw  = target_yaw - gyro_z_dps;
  integral_yaw    += error_yaw * dt;
  integral_yaw     = constrain(integral_yaw, -I_LIMIT, I_LIMIT);
  float out_yaw    = (Kp_yaw * error_yaw)
                   + (Ki_yaw * integral_yaw);

  // ----------------------------------------------------------
  //  7. MOTOR MIXING (Quad-X layout)
  // ----------------------------------------------------------
  int m1 = (int)throttle - (int)out_roll - (int)out_pitch - (int)out_yaw;  // FL CCW
  int m2 = (int)throttle - (int)out_roll + (int)out_pitch + (int)out_yaw;  // BL CW
  int m3 = (int)throttle + (int)out_roll - (int)out_pitch + (int)out_yaw;  // FR CW
  int m4 = (int)throttle + (int)out_roll + (int)out_pitch - (int)out_yaw;  // BR CCW

  // ----------------------------------------------------------
  //  8. SAFETY & OUTPUT
  // ----------------------------------------------------------
  if (armed && throttle > THROTTLE_ARM) {
    motors[0].writeMicroseconds(constrain(m1, 1000, 2000));
    motors[1].writeMicroseconds(constrain(m2, 1000, 2000));
    motors[2].writeMicroseconds(constrain(m3, 1000, 2000));
    motors[3].writeMicroseconds(constrain(m4, 1000, 2000));

    //Serial.print("M1: ");    Serial.print(m1);  Serial.print(" us\t");
    //Serial.print("M2: ");    Serial.print(m2);  Serial.print(" us\t");
    //Serial.print("M3: ");    Serial.print(m3);  Serial.print(" us\t");
    //Serial.print("M4: ");    Serial.print(m4);  Serial.print(" us\t");
    //Serial.println();
    //Serial.printf("R:%.2f P:%.2f Y:%.2f | tgt_R:%.2f tgt_P:%.2f\n",
    //            roll, pitch, yaw, target_roll, target_pitch);

  } else {
    // Disarmed or throttle too low: all motors off, reset integrals
    for (int i = 0; i < 4; i++) motors[i].writeMicroseconds(1000);
    integral_roll = integral_pitch = integral_yaw = 0.0f;
  }
}
