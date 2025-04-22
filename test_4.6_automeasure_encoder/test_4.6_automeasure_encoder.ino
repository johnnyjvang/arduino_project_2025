// ----- Pin Definitions -----
#define ENCODER_L_A 2  // Left encoder A signal
#define MOTOR_L_IN1 7   // Left motor direction pin 1
#define MOTOR_L_IN2 8   // Left motor direction pin 2
#define MOTOR_L_PWM 9   // Left motor PWM pin

// ----- Variables -----
volatile long encoderLeftCount = 0;
unsigned long startTime = 0;
bool measured = false;

// ----- Interrupt Service Routine -----
void encoderLeftISR() {
  encoderLeftCount++;
}

void setup() {
  Serial.begin(115200);

  // Setup encoder
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);

  // Setup motor
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);

  // Start motor forward slowly
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_PWM, 100);  // LOW SPEED to avoid overshoot

  startTime = millis();
}

void loop() {
  // Run for about 2 seconds
  if (!measured && (millis() - startTime) > 2000) {
    // Stop motor
    analogWrite(MOTOR_L_PWM, 0);
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);

    // Print out how many counts it got
    Serial.println("===== Measurement Complete =====");
    Serial.print("Encoder counts in ~2 seconds: ");
    Serial.println(encoderLeftCount);
    Serial.println("Estimate: manually observe how many turns occurred and calculate counts/rev.");

    measured = true; // Only do this once
  }
}
