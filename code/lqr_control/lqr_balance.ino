#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

// =====================================================
//                     MOTOR PINS
// =====================================================
// Arduino Mega PWM-capable pins used here: 4, 5, 6, 7
#define AIN1 4
#define AIN2 5
#define BIN1 7
#define BIN2 6

MPU6050 mpu;

// =====================================================
//                     RAW DATA
// =====================================================
int16_t ax, ay, az, gx, gy, gz;

// =====================================================
//                  ANGLE VARIABLES
// =====================================================
float accAngle = 0.0;
float gyroRate = 0.0;
float angle = 0.0;

// =====================================================
//                KALMAN VARIABLES
// =====================================================
float Q_angle   = 0.001;
float Q_bias    = 0.003;
float R_measure = 0.03;

float angle_k = 0.0;
float bias = 0.0;
float P[2][2] = {{0.0, 0.0}, {0.0, 0.0}};

// =====================================================
//                  LQR GAINS
// =====================================================
// Control law: u = -(K1*theta + K2*theta_dot)
//
// These are practical starting gains, not exact model-derived gains.
// You will likely need to tune them experimentally.
float K1 = 22.0;   // angle gain
float K2 = 1.2;    // angular velocity gain

// =====================================================
//                 GYRO CALIBRATION
// =====================================================
float gyroYOffset = 0.0;

// =====================================================
//                    TIMING
// =====================================================
unsigned long prevTime = 0;

// =====================================================
//                  SAFETY LIMITS
// =====================================================
const int MAX_PWM = 180;        // safer than 255 during first testing
const int DEADBAND = 30;        // motor deadzone compensation
const float FALL_ANGLE = 35.0;  // stop motors if robot tilts too much

// =====================================================
//                KALMAN FILTER FUNCTION
// =====================================================
float getKalmanAngle(float newAngle, float newRate, float dt) {
  // Prediction
  angle_k += dt * (newRate - bias);

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
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

// =====================================================
//                   STOP MOTORS
// =====================================================
void stopMotors() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

// =====================================================
//                MOTOR CONTROL FUNCTION
// =====================================================
// This assumes each motor is driven using two control pins.
// If one wheel rotates in the wrong direction, swap its mapping.
void setMotors(int speed) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  // Deadband compensation
  if (speed > 0) {
    speed += DEADBAND;
    if (speed > MAX_PWM) speed = MAX_PWM;
  } else if (speed < 0) {
    speed -= DEADBAND;
    if (speed < -MAX_PWM) speed = -MAX_PWM;
  }

  int pwm = abs(speed);

  // LEFT MOTOR
  if (speed > 0) {
    analogWrite(AIN1, pwm);
    analogWrite(AIN2, 0);
  } 
  else if (speed < 0) {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, pwm);
  } 
  else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
  }

  // RIGHT MOTOR (reversed relative to left)
  if (speed > 0) {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, pwm);
  } 
  else if (speed < 0) {
    analogWrite(BIN1, pwm);
    analogWrite(BIN2, 0);
  } 
  else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
}

// =====================================================
//               GYRO OFFSET CALIBRATION
// =====================================================
void calibrateGyro() {
  long sumGy = 0;
  const int samples = 1000;

  Serial.println("Keep robot still... calibrating gyro");

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumGy += gy;
    delay(2);
  }

  gyroYOffset = (float)sumGy / samples;

  Serial.print("Gyro Y offset = ");
  Serial.println(gyroYOffset);
}

// =====================================================
//                       SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  stopMotors();

  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 Connected");
  } else {
    Serial.println("MPU6050 Failed");
    while (1) {
      stopMotors();
    }
  }

  calibrateGyro();

  // Initial angle
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  angle_k = accAngle;
  angle = accAngle;

  prevTime = micros();

  Serial.println("System Ready");
  Serial.println("Place robot near upright and test carefully.");
}

// =====================================================
//                        LOOP
// =====================================================
void loop() {
  // Time
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  if (dt <= 0.0 || dt > 0.05) return; // ignore bad timing spikes

  // Read sensor
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Angle from accelerometer
  // Keep this only if this matches your sensor mounting orientation
  accAngle = atan2((float)ax, (float)az) * 180.0 / PI;

  // Angular velocity from gyro Y-axis
  gyroRate = ((float)gy - gyroYOffset) / 131.0;

  // Kalman filtered angle
  angle = getKalmanAngle(accAngle, gyroRate, dt);

  // =================================================
  //                 LQR CONTROL
  // =================================================
  float theta = angle;
  float theta_dot = gyroRate;

  float output = -(K1 * theta + K2 * theta_dot);

  output = constrain(output, -MAX_PWM, MAX_PWM);

  // =================================================
  //                    SAFETY
  // =================================================
  if (abs(angle) > FALL_ANGLE) {
    stopMotors();
  } else {
    setMotors((int)output);
  }

  // =================================================
  //                  DEBUG PRINT
  // =================================================
  Serial.print("AX: "); Serial.print(ax);
  Serial.print(" AY: "); Serial.print(ay);
  Serial.print(" AZ: "); Serial.print(az);

  Serial.print(" | GX: "); Serial.print(gx);
  Serial.print(" GY: "); Serial.print(gy);

  Serial.print(" | accAngle: "); Serial.print(accAngle, 2);
  Serial.print(" | gyroRate: "); Serial.print(gyroRate, 2);
  Serial.print(" | Angle: "); Serial.print(angle, 2);

  Serial.print(" | Out: "); Serial.println((int)output);

  delay(5);  // around 200 Hz loop, helps make timing smoother
}
