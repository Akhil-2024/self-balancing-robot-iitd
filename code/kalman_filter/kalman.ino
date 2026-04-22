#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

MPU6050 mpu;

// Raw sensor data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Angle variables
float accAngle = 0.0;
float gyroRate = 0.0;
float kalmanAngle = 0.0;

// Kalman filter variables
float Q_angle = 0.001;     // Process noise for angle
float Q_bias  = 0.003;     // Process noise for gyro bias
float R_measure = 0.03;    // Measurement noise

float angle = 0.0;         // Estimated angle
float bias = 0.0;          // Estimated gyro bias
float rate = 0.0;          // Unbiased gyro rate

float P[2][2] = { {0, 0}, {0, 0} };

// Timing
unsigned long prevTime = 0;

// ---------------- KALMAN FILTER FUNCTION ----------------
float getKalmanAngle(float newAngle, float newRate, float dt) {
  // Prediction step
  rate = newRate - bias;
  angle += dt * rate;

  // Update estimation error covariance
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Innovation
  float S = P[0][0] + R_measure;

  // Kalman gain
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Angle difference
  float y = newAngle - angle;

  // Update angle and bias
  angle += K[0] * y;
  bias  += K[1] * y;

  // Update error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();

  Serial.println("Reading MPU6050 data with Kalman filter...");

  // Read initial values once for proper initialization
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer angle initialization
  accAngle = atan2(ay, az) * 180.0 / PI;
  angle = accAngle;
  kalmanAngle = accAngle;

  prevTime = millis();
}

void loop() {
  // Read raw MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Prevent dt = 0
  if (dt <= 0) dt = 0.01;

  // ---------------- ACCELEROMETER ANGLE ----------------
  // Tilt angle from accelerometer
  accAngle = atan2(ay, az) * 180.0 / PI;

  // ---------------- GYROSCOPE RATE ----------------
  // MPU6050 sensitivity = 131 LSB/deg/s for ±250 deg/s
  gyroRate = gx / 131.0;

  // ---------------- KALMAN FILTER ----------------
  kalmanAngle = getKalmanAngle(accAngle, gyroRate, dt);

  // ---------------- SERIAL OUTPUT ----------------
  Serial.print("AX: ");
  Serial.print(ax);
  Serial.print(" AY: ");
  Serial.print(ay);
  Serial.print(" AZ: ");
  Serial.print(az);

  Serial.print(" | GX: ");
  Serial.print(gx);
  Serial.print(" GY: ");
  Serial.print(gy);
  Serial.print(" GZ: ");
  Serial.print(gz);

  Serial.print(" | Acc Angle: ");
  Serial.print(accAngle);

  Serial.print(" | Gyro Rate: ");
  Serial.print(gyroRate);

  Serial.print(" | Kalman Angle: ");
  Serial.println(kalmanAngle);

  delay(50);
}
