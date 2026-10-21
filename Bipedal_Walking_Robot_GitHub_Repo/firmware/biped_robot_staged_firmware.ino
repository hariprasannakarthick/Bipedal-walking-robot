/*
  Biped robot staged commissioning firmware
  Controller: Teensy 4.0
  Servo driver: PCA9685
  IMU: MPU6050
  Six nominal 15 kg.cm servos; actual model/voltage must be verified.

  SAFETY:
  - This is NOT an autonomous walking controller.
  - Servo outputs start DISARMED.
  - Test with robot restrained and physical power cutoff available.
  - Confirm servo voltage rating before connecting 7.4-8.4 V.
  - Replace pulse limits and joint map only after measuring each mechanism.
  - Do not power servos from Teensy 3.3V.

  Libraries:
    Adafruit PWM Servo Driver Library
    Adafruit MPU6050
    Adafruit Unified Sensor
    Adafruit BusIO
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_MPU6050 mpu;

static const uint8_t SERVO_COUNT = 6;
static const uint16_t PWM_FREQ_HZ = 50;

// Generic pulse values only for initial low-risk commissioning.
// Verify every servo's pulse range. Some servos can be damaged by these limits.
uint16_t pulseMin[SERVO_COUNT] = {120, 120, 120, 120, 120, 120};
uint16_t pulseMax[SERVO_COUNT] = {520, 520, 520, 520, 520, 520};
uint16_t neutralPulse[SERVO_COUNT] = {307, 307, 307, 307, 307, 307};

// Servo direction: +1 or -1 per channel after physical verification.
int8_t servoDirection[SERVO_COUNT] = {+1, +1, +1, -1, -1, -1};

// Provisional channel labels:
// 0 Left hip pitch; 1 Left knee/linkage; 2 Left auxiliary/ankle linkage
// 3 Right hip pitch; 4 Right knee/linkage; 5 Right auxiliary/ankle linkage
// Rename channels 2 and 5 if CAD confirms a different mechanism.

bool servoArmed = false;
bool imuOK = false;
bool imuFault = false;
bool haveAngle = false;

float gyroBiasY = 0.0f;
float pitchDeg = 0.0f;
float accelPitchDeg = 0.0f;
float alpha = 0.98f;

const float TILT_CUTOFF_DEG = 35.0f; // conservative initial software cutoff
const uint32_t CONTROL_PERIOD_US = 10000; // 100 Hz
uint32_t lastControlUs = 0;
uint32_t lastReportMs = 0;

float wrapAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

uint16_t angleToPulse(uint8_t ch, float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  float mapped = (float)pulseMin[ch] +
      (angleDeg / 180.0f) * (float)(pulseMax[ch] - pulseMin[ch]);
  return (uint16_t)constrain((int)lroundf(mapped),
                             (int)pulseMin[ch], (int)pulseMax[ch]);
}

void writeServoAngle(uint8_t ch, float logicalAngle) {
  if (ch >= SERVO_COUNT) return;
  logicalAngle = constrain(logicalAngle, 0.0f, 180.0f);
  float physicalAngle = 90.0f + servoDirection[ch] * (logicalAngle - 90.0f);
  uint16_t pulse = angleToPulse(ch, physicalAngle);
  pwm.setPWM(ch, 0, pulse);
}

void setNeutralPose() {
  if (!servoArmed) return;
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    pwm.setPWM(ch, 0, neutralPulse[ch]);
  }
}

void disarmServos() {
  // PCA9685 full-off bit for each channel.
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
    pwm.setPWM(ch, 0, 4096);
  }
  servoArmed = false;
}

bool readPitch(float dt) {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  if (!isfinite(accel.acceleration.x) || !isfinite(accel.acceleration.z) ||
      !isfinite(gyro.gyro.y)) {
    imuFault = true;
    return false;
  }

  // Assumes X is the forward-axis acceleration component and Z is vertical.
  // If the IMU is mounted differently, change axes and verify sign experimentally.
  accelPitchDeg = atan2f(accel.acceleration.x, accel.acceleration.z) * 180.0f / PI;

  // Adafruit gyro values are radians/sec. Convert the selected Y axis to deg/sec.
  // Confirm axis and sign by hand-tilting the assembled robot.
  float gyroRateDegS = gyro.gyro.y * 180.0f / PI - gyroBiasY;

  if (!haveAngle) {
    pitchDeg = accelPitchDeg;
    haveAngle = true;
  } else {
    pitchDeg = alpha * (pitchDeg + gyroRateDegS * dt)
             + (1.0f - alpha) * accelPitchDeg;
  }
  pitchDeg = wrapAngle(pitchDeg);
  return isfinite(pitchDeg);
}

void calibrateGyroBias() {
  const int samples = 500;
  double sumY = 0.0;
  sensors_event_t accel, gyro, temp;

  Serial.println("Keep robot stationary: calibrating gyro bias...");
  delay(500);
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accel, &gyro, &temp);
    if (isfinite(gyro.gyro.y)) {
      sumY += (double)(gyro.gyro.y * 180.0f / PI);
    }
    delay(4);
  }
  gyroBiasY = (float)(sumY / samples);
  Serial.print("Gyro Y bias (deg/s): ");
  Serial.println(gyroBiasY, 4);
}

void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  h                 help");
  Serial.println("  s                 status");
  Serial.println("  d                 disarm all servo outputs");
  Serial.println("  a                 arm outputs (only when supported/clear)");
  Serial.println("  n                 command stored neutral pulse pose");
  Serial.println("  j <ch> <deg>      command one servo, channel 0-5, logical 0-180");
  Serial.println("  p <ch> <ticks>    set one channel pulse ticks, within configured limits");
  Serial.println("  c                 recalibrate gyro bias (robot stationary)");
  Serial.println("No walking command is implemented in this commissioning sketch.");
}

void printStatus() {
  Serial.print("IMU_OK="); Serial.print(imuOK);
  Serial.print(" IMU_FAULT="); Serial.print(imuFault);
  Serial.print(" ARMED="); Serial.print(servoArmed);
  Serial.print(" pitch_deg="); Serial.print(pitchDeg, 2);
  Serial.print(" accel_pitch_deg="); Serial.print(accelPitchDeg, 2);
  Serial.print(" gyro_bias_y_deg_s="); Serial.println(gyroBiasY, 3);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  if (line == "h") { printHelp(); return; }
  if (line == "s") { printStatus(); return; }
  if (line == "d") { disarmServos(); Serial.println("DISARMED"); return; }
  if (line == "c") {
    if (servoArmed) {
      Serial.println("Disarm before calibrating.");
      return;
    }
    calibrateGyroBias();
    return;
  }
  if (line == "a") {
    if (!imuOK || imuFault || !haveAngle) {
      Serial.println("Refused: IMU not healthy.");
      return;
    }
    if (fabsf(pitchDeg) > 15.0f) {
      Serial.println("Refused: hold robot near calibrated upright angle before arming.");
      return;
    }
    servoArmed = true;
    setNeutralPose();
    Serial.println("Outputs armed at stored neutral pulses. Keep robot supported.");
    return;
  }
  if (line == "n") {
    if (!servoArmed) { Serial.println("Arm first."); return; }
    setNeutralPose();
    Serial.println("Neutral pulses commanded.");
    return;
  }

  int ch = -1;
  float value = 0.0f;
  if (sscanf(line.c_str(), "j %d %f", &ch, &value) == 2) {
    if (!servoArmed) { Serial.println("Arm first."); return; }
    if (ch < 0 || ch >= SERVO_COUNT) { Serial.println("Invalid channel."); return; }
    writeServoAngle((uint8_t)ch, value);
    Serial.println("Servo command sent. Inspect linkage before continuing.");
    return;
  }
  int ticks = -1;
  if (sscanf(line.c_str(), "p %d %d", &ch, &ticks) == 2) {
    if (!servoArmed) { Serial.println("Arm first."); return; }
    if (ch < 0 || ch >= SERVO_COUNT) { Serial.println("Invalid channel."); return; }
    if (ticks < pulseMin[ch] || ticks > pulseMax[ch]) {
      Serial.println("Rejected: pulse outside configured bounds.");
      return;
    }
    pwm.setPWM((uint8_t)ch, 0, (uint16_t)ticks);
    Serial.println("Pulse command sent.");
    return;
  }
  Serial.println("Unknown command. Send h for help.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);
  disarmServos();

  if (mpu.begin(0x68, &Wire)) {
    imuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    calibrateGyroBias();
  } else {
    imuOK = false;
    imuFault = true;
    Serial.println("ERROR: MPU6050 not found at address 0x68.");
  }

  lastControlUs = micros();
  Serial.println("\nBiped staged commissioning firmware ready.");
  Serial.println("Servo outputs are DISARMED. Keep robot supported.");
  printHelp();
}

void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    float dt = (nowUs - lastControlUs) / 1000000.0f;
    lastControlUs = nowUs;

    if (imuOK && !readPitch(dt)) {
      imuFault = true;
      disarmServos();
    }

    if (!isfinite(pitchDeg) || fabsf(pitchDeg) > TILT_CUTOFF_DEG) {
      imuFault = true;
      disarmServos();
    }
  }

  if (millis() - lastReportMs >= 250) {
    lastReportMs = millis();
    printStatus();
  }
}
