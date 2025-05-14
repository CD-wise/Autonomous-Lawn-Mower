
#include <AFMotor.h> // Library for the L293D Motor Driver Shield
#include <Servo.h>   // Library for Servo Motor

// Ultrasonic Sensor Pins
#define TRIG_PIN A5
#define ECHO_PIN A4

// Initialize Motors
AF_DCMotor leftMotor(3);  // Motor 3 (M3 on the shield)
AF_DCMotor rightMotor(2); // Motor 2 (M2 on the shield)
AF_DCMotor rightLMotor(1); // Motor 1 (M1 on the shield)
// Initialize Servo
Servo myServo;

// Distance threshold for obstacle detection
const int DISTANCE_THRESHOLD = 20; // in centimeters

void setup() {
  // Serial Monitor for Debugging
  Serial.begin(9600);

  // Ultrasonic Sensor Setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo Initialization
  myServo.attach(9); // Servo connected to pin 9
  myServo.write(90); // Servo at initial position (facing forward)

  // Motors Initialization
  leftMotor.setSpeed(150);  // Speed range: 0-255
  rightMotor.setSpeed(150); // Speed range: 0-255
  rightLMotor.setSpeed(150);
  Serial.println("Obstacle Avoidance Robot Initialized");
}

void loop() {
  // Get distance from Ultrasonic Sensor
  int distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > DISTANCE_THRESHOLD) {
    moveForward();
  } else {
    avoidObstacle();
  }

  delay(100);
}

// Function to measure distance using Ultrasonic Sensor
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2; // Convert to centimeters
  return distance;
}

// Movement Functions
void moveForward() {
  Serial.println("Moving Forward");
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  rightLMotor.run(FORWARD);
}

void moveBackward() {
  Serial.println("Moving Backward");
  leftMotor.run(BACKWARD);
  rightMotor.run(BACKWARD);
  rightLMotor.run(BACKWARD);
}

void turnLeft() {
  Serial.println("Turning Left");
  leftMotor.run(BACKWARD);
  rightMotor.run(FORWARD);
  rightLMotor.run(FORWARD);
}

void turnRight() {
  Serial.println("Turning Right");
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
  rightLMotor.run(BACKWARD);
}

void stopMotors() {
  Serial.println("Stopping Motors");
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  rightLMotor.run(RELEASE);
}

// Obstacle Avoidance Logic
void avoidObstacle() {
  stopMotors();
  delay(500);

  // Scan surroundings with the Servo
  myServo.write(30); // Look left
  delay(500);
  int distanceLeft = getDistance();

  myServo.write(180); // Look right
  delay(500);
  int distanceRight = getDistance();

  myServo.write(90); // Reset to center
  delay(500);

  // Determine the best direction
  if (distanceLeft > distanceRight && distanceLeft > DISTANCE_THRESHOLD) {
    turnLeft();
    delay(1000); // Adjust turning duration
  } else if (distanceRight > distanceLeft && distanceRight > DISTANCE_THRESHOLD) {
    turnRight();
    delay(1000); // Adjust turning duration
  } else {
    moveBackward();
    delay(1000); // Move back to create space
  }
}
