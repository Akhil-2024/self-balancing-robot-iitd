#define IN1 4
#define IN2 5
#define IN3 7
#define IN4 6

int speedVal = 180;  // Speed (0–255)

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);   // Start Serial Monitor
  Serial.println("Motor Test Started...");
}

// ------------------ MOTOR FUNCTIONS ------------------

void moveForward() {
  Serial.println("Moving Forward");
  Serial.print("Speed: "); Serial.println(speedVal);

  analogWrite(IN1, speedVal);
  analogWrite(IN2, 0);
  analogWrite(IN3, speedVal);
  analogWrite(IN4, 0);
}

void moveBackward() {
  Serial.println("Moving Backward");
  Serial.print("Speed: "); Serial.println(speedVal);

  analogWrite(IN1, 0);
  analogWrite(IN2, speedVal);
  analogWrite(IN3, 0);
  analogWrite(IN4, speedVal);
}

void turnLeft() {
  Serial.println("Turning Left");

  analogWrite(IN1, 0);
  analogWrite(IN2, speedVal);
  analogWrite(IN3, speedVal);
  analogWrite(IN4, 0);
}

void turnRight() {
  Serial.println("Turning Right");

  analogWrite(IN1, speedVal);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, speedVal);
}

void stopMotors() {
  Serial.println("Motors Stopped");

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

// ------------------ LOOP ------------------

void loop() {

  Serial.println("---- New Cycle ----");

  moveForward();
  delay(3000);

  stopMotors();
  delay(1000);

  moveBackward();
  delay(3000);

  stopMotors();
  delay(1000);

  turnLeft();
  delay(2000);

  stopMotors();
  delay(1000);

  turnRight();
  delay(2000);

  stopMotors();
  delay(2000);
}
