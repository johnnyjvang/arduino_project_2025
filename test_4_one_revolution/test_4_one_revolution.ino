// ----- Pin Definitions -----
#define ENCODER_L_A 2  // Left encoder A signal
#define ENCODER_L_B 3  // Left encoder B signal
#define MOTOR_L_IN1 7   // Left motor direction pin 1
#define MOTOR_L_IN2 8   // Left motor direction pin 2
#define MOTOR_L_PWM 9   // Left motor PWM pin

// ----- Variables -----
volatile long encoderLeftCount = 0;
const int countsPerRevolution = 11; // <<< CHANGE this number after you measure!!

// ----- Interrupt Service Routine -----
void encoderLeftISR() {
  encoderLeftCount++;
}

void setup() {
  Serial.begin(115200);

  // Setup encoder pins
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);

  // Setup motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);

  // Reset encoder
  encoderLeftCount = 0;

  // Set motor forward
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  
  analogWrite(MOTOR_L_PWM, 100); // Start motor at moderate speed
}

void loop() {
  Serial.print("Encoder Count: ");
  Serial.println(encoderLeftCount);

  if (encoderLeftCount >= countsPerRevolution) {
    // Stop motor
    analogWrite(MOTOR_L_PWM, 0);
    
    Serial.println("One revolution complete!");
    
    while (1) {
      // Stop forever after 1 turn
    }
  }

  delay(50); // Check every 50ms
}
