int enA = 9;  // Enable pin (PWM for speed control)
int in1 = 8;  // Control pin 1
int in2 = 7;  // Control pin 2

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Set motor direction (Forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Set speed (0-255)
  analogWrite(enA, 60);  // Full speed

  delay(1000); // Run for 3 seconds

  // Stop motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  delay(5000); // Wait for 2 seconds
}