#include <NewPing.h>  // Library for ultrasonic sensors

#define VA 220
#define VB 255
// Pin definitions
#define TRIGGER_PIN_FRONT 13  // Trigger pin for front sensor
#define ECHO_PIN_FRONT 2     // Echo pin for front sensor
#define TRIGGER_PIN_LEFT 13   // Trigger pin for left sensor
#define ECHO_PIN_LEFT 12      // Echo pin for left sensor
#define TRIGGER_PIN_RIGHT 13  // Trigger pin for right sensor
#define ECHO_PIN_RIGHT 7     // Echo pin for right sensor
#define ENA 11         // Motor A enable pin
#define IN_A_1 10        // Motor A input 1
#define IN_A_2 9        // Motor A input 2
#define ENB 3         // Motor B enable pin
#define IN_B_1 5         // Motor B input 3
#define IN_B_2 6         // Motor B input 4

// Robot dimensions
#define ROBOT_WIDTH 22.5       // Width of the robot in cm
#define ROBOT_LENGTH 30    // Length of the robot in cm

// Ultrasonic sensor objects
NewPing frontSensor(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, 500);  // Front sensor
NewPing leftSensor(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, 500);      // Left sensor
NewPing rightSensor(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, 500);   // Right sensor

// Movement variables
float linearVelocity = 28.99 * 0.001;   // Linear velocity in cm/ms
float angularVelocity = 105.07 * 0.001; // Angular velocity in degrees/ms


void backward(float distance=0) {
  analogWrite(ENA, VA);
  analogWrite(ENB, VB);

  digitalWrite(IN_A_1, HIGH);
  digitalWrite(IN_A_2, LOW);

  digitalWrite(IN_B_1, HIGH);
  digitalWrite(IN_B_2, LOW);

  if(distance!=0){
    delay(distance/linearVelocity);
  }
}

void forward(float distance=0) {
  analogWrite(ENA, VA);
  analogWrite(ENB, VB);

  digitalWrite(IN_A_1, LOW);
  digitalWrite(IN_A_2, HIGH);

  digitalWrite(IN_B_1, LOW);
  digitalWrite(IN_B_2, HIGH);

  if(distance!=0){
    delay(distance/linearVelocity);
  }
}

void right(float angle=0) {
  analogWrite(ENA, VA);
  analogWrite(ENB, VB);

  digitalWrite(IN_A_1, HIGH);
  digitalWrite(IN_A_2, LOW);

  digitalWrite(IN_B_1, LOW);
  digitalWrite(IN_B_2, HIGH);

  if(angle!=0){
    delay(angle/angularVelocity);
  }
}

void left(float angle=0) {
  analogWrite(ENA, VA);
  analogWrite(ENB, VB);

  digitalWrite(IN_A_1, LOW);
  digitalWrite(IN_A_2, HIGH);

  digitalWrite(IN_B_1, HIGH);
  digitalWrite(IN_B_2, LOW);

  if(angle!=0){
    delay(angle/angularVelocity);
  }
}

void stationary() {
  digitalWrite(IN_A_1, LOW);
  digitalWrite(IN_A_2, LOW);

  digitalWrite(IN_B_1, LOW);
  digitalWrite(IN_B_2, LOW);
}


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN_A_1, OUTPUT);
  pinMode(IN_A_2, OUTPUT);

  pinMode(IN_B_1, OUTPUT);
  pinMode(IN_B_2, OUTPUT);
}

void loop() {
  // Read distance from front sensor
  int frontDistance = frontSensor.ping_cm();

  // Read distances from left and right sensors
  int leftDistance = leftSensor.ping_cm();
  int rightDistance = rightSensor.ping_cm();

  // Print sensor readings
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Obstacle detection and avoidance logic
  if (frontDistance > 0 && frontDistance <= 30) {
    if (rightDistance > 0 && rightDistance <= 30) {
      // If obstacles are detected in both front and right, move backward and turn left
      backward(15);
      left(90);
      forward(50);
      right(90);
    } else {
      // If only an obstacle is detected in front, turn right
      right(90);
      forward(50);
      left(90);
    }
  } else if (leftDistance > 0 && leftDistance <= 30) {
    // If an obstacle is detected on the left, turn right
      right(60);
      forward(50);
      left(60);
      Serial.println("L");
  } else if (rightDistance > 0 && rightDistance <= 30) {
    // If an obstacle is detected on the right, turn left
      left(60);
      forward(50);
      right(60);
      Serial.println("R");
  } else {
    // No obstacles detected, move forward
    forward();
  }
}
