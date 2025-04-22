// ----- Pin Definitions -----
#define ENCODER_L_A 2  // Left encoder A signal

// ----- Variables -----
volatile long encoderLeftCount = 0;

// ----- Interrupt Service Routine -----
void encoderLeftISR() {
  encoderLeftCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, RISING);
}

void loop() {
  Serial.print("Encoder Count: ");
  Serial.println(encoderLeftCount);
  delay(500); // Every 0.5 seconds
}
