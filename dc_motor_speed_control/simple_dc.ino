int motorPin = 3;

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
   digitalWrite(motorPin, HIGH);
}