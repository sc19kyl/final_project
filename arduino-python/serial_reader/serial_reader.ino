const int ledPin = 9; // PWM pin

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Same baud rate as defined in Python code
}

void loop() {
  if (Serial.available() > 0) {
    int pwmValue = Serial.read();
    Serial.print(pwmValue); // Write PWM value to LED
  }
}
