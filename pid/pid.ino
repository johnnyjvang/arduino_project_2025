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

// ----- Constants -----
const int ENCODER_COUNTS_PER_REV = 105; // Adjust based on actual encoder counts per rev
const int SAMPLE_TIME = 100; // Sampling time in ms

// ----- PID Constants -----
float Kp = 1.2;  // Proportional Gain
float Ki = 0.5;  // Integral Gain
float Kd = 0.05; // Derivative Gain

// ----- Variables -----
volatile int encoderLeftCount = 0;
volatile int encoderRightCount = 0;
float prevErrorL = 0, prevErrorR = 0;
float integralL = 0, integralR = 0;
float motorPWM_L = 0, motorPWM_R = 0;
float targetSpeed_L = 0, targetSpeed_R = 0;

// ----- Interrupt Service Routines -----
void encoderLeftISR() {
  encoderLeftCount++;
}
void encoderRightISR() {
  encoderRightCount++;
}

// ----- Setup Function -----
void setup() {
  Serial.begin(115200);

  // Setup Encoders
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderRightISR, RISING);

  // Setup Motor Control Pins
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
}

// ----- Function to Compute RPM -----
float calculateRPM(volatile int &encoderCount) {
  float rotations = (float)encoderCount / ENCODER_COUNTS_PER_REV;
  float rpm = (rotations * 60000.0) / SAMPLE_TIME;
  encoderCount = 0; // Reset count for next sample
  return rpm;
}

// ----- PID Motor Control -----
void updateMotorSpeed(float targetL, float targetR) {
  // If both speeds are 0, stop the motors completely
  if (targetL == 0 && targetR == 0) {
    analogWrite(MOTOR_L_PWM, 0);  // Set PWM to 0
    analogWrite(MOTOR_R_PWM, 0);  // Set PWM to 0
    digitalWrite(MOTOR_L_IN1, LOW);  // Ensure direction pins are LOW
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    return;  // Exit function to stop motors
  }

  float actualL = calculateRPM(encoderLeftCount);
  float actualR = calculateRPM(encoderRightCount);

  float errorL = targetL - actualL;
  float errorR = targetR - actualR;

  integralL += errorL * (SAMPLE_TIME / 1000.0);
  integralR += errorR * (SAMPLE_TIME / 1000.0);

  float derivativeL = (errorL - prevErrorL) / (SAMPLE_TIME / 1000.0);
  float derivativeR = (errorR - prevErrorR) / (SAMPLE_TIME / 1000.0);

  motorPWM_L = (Kp * errorL) + (Ki * integralL) + (Kd * derivativeL);
  motorPWM_R = (Kp * errorR) + (Ki * integralR) + (Kd * derivativeR);

  // Add minimum PWM to prevent stalling
  if (motorPWM_L > 0) motorPWM_L = constrain(motorPWM_L, 10, 255);  // Set minimum PWM to avoid stalling
  else motorPWM_L = constrain(motorPWM_L, -255, -10);

  if (motorPWM_R > 0) motorPWM_R = constrain(motorPWM_R, 10, 255);  // Set minimum PWM to avoid stalling
  else motorPWM_R = constrain(motorPWM_R, -255, -10);

  // Apply motor control
  digitalWrite(MOTOR_L_IN1, targetL >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_L_IN2, targetL >= 0 ? LOW : HIGH);
  analogWrite(MOTOR_L_PWM, abs(motorPWM_L));

  digitalWrite(MOTOR_R_IN1, targetR >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_IN2, targetR >= 0 ? LOW : HIGH);
  analogWrite(MOTOR_R_PWM, abs(motorPWM_R));

  // Debug print
  Serial.print("Target L: "); Serial.print(targetL);
  Serial.print(" | Actual L: "); Serial.print(actualL);
  Serial.print(" | Enc L: "); Serial.print(encoderLeftCount);
  Serial.print(" | PWM L: "); Serial.print(motorPWM_L);
  Serial.print(" || Target R: "); Serial.print(targetR);
  Serial.print(" | Actual R: "); Serial.print(actualR);
  Serial.print(" | Enc R: "); Serial.print(encoderRightCount);
  Serial.print(" | PWM R: "); Serial.println(motorPWM_R);

  prevErrorL = errorL;
  prevErrorR = errorR;
}

// ----- Serial Command Processing -----
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("m ")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex != -1) {
        String values = command.substring(spaceIndex + 1);
        int commaIndex = values.indexOf(',');
        
        if (commaIndex != -1) {
          targetSpeed_L = values.substring(0, commaIndex).toFloat();
          targetSpeed_R = values.substring(commaIndex + 1).toFloat();
        }
      }
    }
  }
}

// ----- Main Loop -----
void loop() {
  processSerialCommand();
  updateMotorSpeed(targetSpeed_L, targetSpeed_R);
  delay(SAMPLE_TIME);
}
