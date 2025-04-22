// ----- Pin Definitions -----

#define ENCODER_A 2      // Encoder A signal pin (Interrupt)
#define ENCODER_B 3      // Encoder B signal pin
#define MOTOR_PWM 9      // Motor PWM control pin
#define MOTOR_IN1 7      // Motor direction control pin 1
#define MOTOR_IN2 8      // Motor direction control pin 2

// ----- Constants -----
const int ENCODER_COUNTS_PER_REV = 105; // Based on your measured value
const int SAMPLE_TIME = 100; // Sampling time in milliseconds
const int TARGET_RPM = 300; // Target speed (adjust as needed)

// ----- PID Constants (Tune these) -----
float Kp = 0.8;  // Reduced Proportional Gain to smooth response
float Ki = 0.4;  // Integral Gain to eliminate steady-state error
float Kd = 0.02; // Lower Derivative Gain to prevent excessive correction

// ----- PID Variables -----
volatile int encoderCount = 0;
float previousError = 0;
float integral = 0;
float motorPWM = 0;

// ----- Interrupt Service Routine (ISR) -----
void encoderISR() {
  encoderCount++;  // Increment encoder pulse count
}

// ----- Setup Function -----
void setup() {
  Serial.begin(115200);
  
  // Setup Encoder Pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Setup Motor Control Pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Attach Interrupt for Encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  // Initialize motor direction
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

// ----- Function to Compute RPM -----
float calculateRPM() {
  float rotations = (float)encoderCount / ENCODER_COUNTS_PER_REV;
  float rpm = (rotations * 60000.0) / SAMPLE_TIME; // Convert to RPM
  encoderCount = 0; // Reset count for next sample
  return rpm;
}

// ----- PID Control Function -----
void updateMotorSpeed(float targetRPM) {
  float actualRPM = calculateRPM();
  float error = targetRPM - actualRPM;

  // PID calculations
  integral += error * (SAMPLE_TIME / 1000.0); // Integral term
  float derivative = (error - previousError) / (SAMPLE_TIME / 1000.0); // Derivative term

  // Compute PID Output
  motorPWM = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Constrain motor speed between 0-255
  motorPWM = constrain(motorPWM, 0, 255);

  // Apply motor control
  analogWrite(MOTOR_PWM, (int)motorPWM);

  // Debug output
  Serial.print("Target RPM: "); Serial.print(targetRPM);
  Serial.print(" | Actual RPM: "); Serial.print(actualRPM);
  Serial.print(" | Motor PWM: "); Serial.println(motorPWM);

  previousError = error; // Store error for next iteration
}

// ----- Main Loop -----
void loop() {
  updateMotorSpeed(TARGET_RPM);  // Maintain target speed
  delay(SAMPLE_TIME); // Wait before next PID update
}
