#include <Servo.h>

// === 1. Definitions ===
// Define pins connected to motors, sensors, servo, and pump

#define enA   5     // Enable pin for Motor A (PWM)
#define in1   6     // Direction control pin IN1 for Motor A
#define in2   7     // Direction control pin IN2 for Motor A
#define in3   8     // Direction control pin IN3 for Motor B
#define in4   9     // Direction control pin IN4 for Motor B
#define enB   10    // Enable pin for Motor B (PWM)

#define ir_R  A0    // IR flame sensor - Right
#define ir_L  A1    // IR flame sensor - Left
#define ir_F  A2    // IR flame sensor - Front

#define servo A5    // Servo control pin
#define pump  2     // Pump control pin (fire extinguisher)


// === 2. Global Variables ===
// Variables used throughout the program

int Speed = 160;    // Motor speed (PWM value from 0 to 255)
int s1, s2, s3;     // IR sensor readings: right, front, and left


// === 3. Helper Functions ===

// Sends a pulse to the servo to move to a specific angle
void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500; // Convert angle to pulse width in microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50); // Small delay for servo movement
}

// Move forward
void forword() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Move backward
void backword() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Turn left
void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Turn right
void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Stop all motors
void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


// === 4. Setup ===
// Runs once at startup

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // Set sensor pins as input
  pinMode(ir_R, INPUT);
  pinMode(ir_F, INPUT);
  pinMode(ir_L, INPUT);

  // Set motor control pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  // Set servo and pump pins as output
  pinMode(servo, OUTPUT);
  pinMode(pump, OUTPUT);

  // Set initial motor speed using PWM
  analogWrite(enA, Speed);
  analogWrite(enB, Speed);

  // Servo sweep (initial test or scanning motion)
  for (int angle = 90; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 40; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 40; angle <= 95; angle += 5) {
    servoPulse(servo, angle);
  }

  delay(500);
}


// === 5. Main Loop ===
// Continuously checks sensors and acts accordingly

void loop() {
  // Read values from the IR flame sensors
  s1 = digitalRead(ir_R);  // Right sensor
  s2 = digitalRead(ir_F);  // Front sensor
  s3 = digitalRead(ir_L);  // Left sensor

  // Display sensor readings in Serial Monitor (for debugging)
  Serial.print(s1); Serial.print("\t");
  Serial.print(s2); Serial.print("\t");
  Serial.println(s3);
  delay(50);

  // === Fire Detected ===

  if (s1 == LOW) {  // Fire detected on the right
    Stop();
    digitalWrite(pump, HIGH);  // Turn on pump

    for (int angle = 90; angle >= 40; angle -= 3) {
      servoPulse(servo, angle);
    }
    for (int angle = 40; angle <= 90; angle += 3) {
      servoPulse(servo, angle);
    }
  }
  else if (s2 == LOW) {  // Fire detected in front
    Stop();
    digitalWrite(pump, HIGH);

    for (int angle = 90; angle <= 140; angle += 3) {
      servoPulse(servo, angle);
    }
    for (int angle = 140; angle >= 40; angle -= 3) {
      servoPulse(servo, angle);
    }
    for (int angle = 40; angle <= 90; angle += 3) {
      servoPulse(servo, angle);
    }
  }
  else if (s3 == LOW) {  // Fire detected on the left
    Stop();
    digitalWrite(pump, HIGH);

    for (int angle = 90; angle <= 140; angle += 3) {
      servoPulse(servo, angle);
    }
    for (int angle = 140; angle >= 90; angle -= 3) {
      servoPulse(servo, angle);
    }
  }

  // === No Fire Detected: Navigate ===

  else if (s1 == HIGH) {
    digitalWrite(pump, LOW);
    backword();
    delay(200);
    turnRight();
    delay(700);
  }
  else if (s2 == HIGH) {
    digitalWrite(pump, LOW);
    forword();
    delay(200);
  }
  else if (s3 == HIGH) {
    digitalWrite(pump, LOW);
    backword();
    delay(200);
    turnLeft();
    delay(700);
  }
  else {
    // If no flame detected, stop movement and turn off pump
    digitalWrite(pump, LOW);
    Stop();
  }

  delay(10); // Small delay before next loop cycle
}