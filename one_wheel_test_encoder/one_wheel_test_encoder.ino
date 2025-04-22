// ----- Pin Definitions -----
#define ENCODER_L_A 2  // Left encoder A signal
#define ENCODER_L_B 3  // Left encoder B signal
#define ENCODER_R_A 18 // Right encoder A signal
#define ENCODER_R_B 19 // Right encoder B signal

#define MOTOR_L_IN1 7   // Left motor direction pin 1
#define MOTOR_L_IN2 8   // Left motor direction pin 2
#define MOTOR_L_PWM 9   // Left motor PWM pin

#define MOTOR_R_IN1 4   // Right motor direction pin 1
#define MOTOR_R_IN2 5   // Right motor direction pin 2
#define MOTOR_R_PWM 6   // Right motor PWM pin

// ----- Variables -----
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

// ----- Interrupt Service Routines -----
void encoderLeftISR() {
  encoderLeftCount++;
}

void encoderRightISR() {
  encoderRightCount++;
}

void setup() {
  Serial.begin(115200);

  // Setup encoder pins
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, RISING);

  // Setup motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);

  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);

  // Set motors to go forward
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 100);  // moderate PWM

  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_PWM, 100);  // moderate PWM
}

void loop() {
  Serial.print("Left Encoder Count: ");
  Serial.print(encoderLeftCount);
  Serial.print("   Right Encoder Count: ");
  Serial.println(encoderRightCount);
  
  delay(500);  // Print every half second
}
