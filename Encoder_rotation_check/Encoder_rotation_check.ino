#define ENCODER_A 2      // Encoder A signal pin (Interrupt)
#define ENCODER_B 3      // Encoder B signal pin
#define MOTOR_PWM 9      // Motor PWM control pin
#define MOTOR_IN1 7      // Motor direction control pin 1
#define MOTOR_IN2 8      // Motor direction control pin 2

volatile int encoderCount = 0;  // Encoder pulse count
volatile int direction = 0;     // 0 = Clockwise, 1 = Counterclockwise
volatile int lastA = LOW;       // Previous state of ENCODER_A

// Encoder Interrupt Service Routine (ISR)
void encoderISR() {
  int A = digitalRead(ENCODER_A);  
  int B = digitalRead(ENCODER_B);  

  if (A != lastA) { // Only process changes
    if (A == HIGH) {  
      // A leading B -> Clockwise
      if (B == LOW) {
        direction = 0;  
      } 
      // A trailing B -> Counterclockwise
      else {
        direction = 1;  
      }
    }
    encoderCount++;  
  }
  lastA = A; // Store last state
}

void setup() {
  Serial.begin(9600);  

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE); // Trigger on any change
}

void runMotor(int speed, bool clockwise) {
  if (clockwise) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  }
  analogWrite(MOTOR_PWM, speed);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
}

void loop() {
  encoderCount = 0;  

  // Run motor clockwise
  runMotor(150, true);
  delay(2000);  
  stopMotor();

  Serial.print("Clockwise: Encoder Count = ");
  Serial.print(encoderCount);
  Serial.print(", Direction = ");
  Serial.println(direction == 0 ? "Clockwise" : "Counterclockwise");

  delay(1000);  

  encoderCount = 0;  

  // Run motor counterclockwise
  runMotor(150, false);
  delay(2000);  
  stopMotor();

  Serial.print("Counterclockwise: Encoder Count = ");
  Serial.print(encoderCount);
  Serial.print(", Direction = ");
  Serial.println(direction == 1 ? "Counterclockwise" : "Clockwise");

  delay(1000);  
}
