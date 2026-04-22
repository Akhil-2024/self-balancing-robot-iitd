#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

// ---------------- MOTOR PINS ----------------
#define AIN1 4
#define AIN2 5
#define BIN1 7
#define BIN2 6

MPU6050 mpu;

// ---------------- RAW DATA ----------------
int16_t ax, ay, az, gx, gy, gz;

// ---------------- ANGLE ----------------
float accAngle, gyroRate, angle;

// ---------------- KALMAN VARIABLES ----------------
float Q_angle = 0.001;
float Q_bias  = 0.003;
float R_measure = 0.03;

float angle_k = 0;
float bias = 0;
float P[2][2] = {{0,0},{0,0}};

// ---------------- PID ----------------
float Kp = 18;
float Ki = 0.5;
float Kd = 1.2;

float setpoint = 0;
float error = 0, prevError = 0;
float integral = 0, derivative = 0;

// ---------------- TIMING ----------------
unsigned long prevTime = 0;

// ---------------- MOTOR DEADZONE ----------------
const int DEADBAND = 35;

// ---------------- KALMAN FUNCTION ----------------
float getKalmanAngle(float newAngle, float newRate, float dt) {

  // Prediction
  angle_k += dt * (newRate - bias);

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update
  float S = P[0][0] + R_measure;

  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle_k;

  angle_k += K[0] * y;
  bias    += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle_k;
}

// ---------------- MOTOR CONTROL ----------------
void setMotors(int speed) {

  // Deadband compensation
  if (speed > 0) speed = constrain(speed + DEADBAND, 0, 255);
  else if (speed < 0) speed = constrain(speed - DEADBAND, -255, 0);

  speed = constrain(speed, -255, 255);

  // LEFT MOTOR
  if (speed < 0) {
  // if (speed > 0) {
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
  } else if (speed > 0) {
  // } else if (speed < 0) {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, -speed);
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
  }

  // RIGHT MOTOR (reversed)
  if (speed < 0) {
  // if (speed > 0) {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
  } else if (speed > 0) {
  // } else if (speed < 0) {
    analogWrite(BIN1, -speed);
    analogWrite(BIN2, 0);
  } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("MPU6050 Failed");
  }

  // Initial angle for Kalman
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accAngle = atan2((float)ax, (float)az) * 180 / PI;
  angle_k = accAngle;

  prevTime = micros();
}

// ---------------- LOOP ----------------
void loop() {

  // Time
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;
  if (dt <= 0) return;

  // Sensor read
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Angle calculation
  accAngle = atan2((float)ax, (float)az) * 180 / PI;
  gyroRate = gy / 131.0;

  angle = getKalmanAngle(accAngle, gyroRate, dt);

  // ---------------- PID ----------------
  error = setpoint - angle;

  if (abs(error) < 15) {
    integral += error * dt;
    integral = constrain(integral, -150, 150);
  } else {
    integral = 0;
  }

  derivative = (error - prevError) / dt;
  prevError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Safety
  if (abs(angle) > 45) {
    setMotors(0);
    integral = 0;
  } else {
    setMotors((int)output);
  }

  // ---------------- DEBUG OUTPUT ----------------
  Serial.print("AX: "); Serial.print(ax);
  Serial.print(" AY: "); Serial.print(ay);
  Serial.print(" AZ: "); Serial.print(az);

  Serial.print(" | GX: "); Serial.print(gx);
  Serial.print(" GY: "); Serial.print(gy);

  Serial.print(" | Angle: "); Serial.print(angle);
  Serial.print(" | Speed: "); Serial.println((int)output);
}
