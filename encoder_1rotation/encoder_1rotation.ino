#define ENCODER_A 2  // Encoder A signal pin (Interrupt)
#define ENCODER_B 3  // Encoder B signal pin
#define MOTOR_PWM 9  // Motor PWM control pin
#define MOTOR_IN1 7  // Motor direction control pin 1
#define MOTOR_IN2 8  // Motor direction control pin 2

volatile int encoderCount = 0;  // Encoder pulse count
const int countsPerRevolution = 105;  // Your measured encoder count per revolution

// Interrupt Service Routine (ISR) to count encoder pulses
void encoderISR() {
  encoderCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // Attach interrupt to Encoder A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
}

void loop() {
  encoderCount = 0;  // Reset encoder count

  // Start motor (Clockwise)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 100);  // Moderate speed

  // Wait until exactly 1 revolution (72 counts)
  while (encoderCount < countsPerRevolution) {
    // Just wait until we reach the target count
  }

  // Stop the motor
  analogWrite(MOTOR_PWM, 0);

  // Print results
  Serial.print("Motor stopped at Encoder Count: ");
  Serial.println(encoderCount);

  delay(5000);  // Pause before next cycle
}
