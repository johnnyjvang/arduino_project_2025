// ----- Motor Pin Definitions -----
#define MOTOR_L_PWM 9   // Left motor PWM pin
#define MOTOR_L_IN1 7   // Left motor direction pin 1
#define MOTOR_L_IN2 8   // Left motor direction pin 2

#define MOTOR_R_PWM 6  // Right motor PWM pin
#define MOTOR_R_IN1 4   // Right motor direction pin 1
#define MOTOR_R_IN2 5  // Right motor direction pin 2

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  // ---- Go FORWARD ----
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 50); // PWM 50

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 50); // PWM 50

  delay(1000); // Move forward for 1 second

  // ---- Go BACKWARD ----
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_L_PWM, 50); // PWM 50

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  analogWrite(MOTOR_R_PWM, 50); // PWM 50

  delay(1000); // Move backward for 1 second

  // ---- STOP Motors ----
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 0);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 0);
}

void loop() {
  // Nothing here
}
