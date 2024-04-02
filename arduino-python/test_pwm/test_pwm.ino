#define enA 9
#define in1 6
#define in2 7
#define enB 10
#define in3 11
#define in4 12

#define ENC_COUNT_REV 620
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2
#define ENC_IN_RIGHT_C 3
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4 
#define ENC_IN_RIGHT_D 5
// True = Forward; False = Reverse
boolean Direction_right = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;

int rotDirection = 0;
int pressed = false;

void setup() {
  Serial.begin(9600);
  pinMode(enA , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_C , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_D , INPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_C), left_wheel_pulse, RISING);
}

void loop() {



  if (Serial.available() > 0) {
    int pwmValue = Serial.read();
    analogWrite(enA, pwmValue); // Write PWM value to LED
  }
    currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;

    rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_left = rpm_left * rpm_to_radians;   
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;
     
    Serial.print(" Speed1: ");
    Serial.print(rpm_right);
    Serial.println(" RPM");

    Serial.print(" Speed2: ");
    Serial.print(rpm_left);
    Serial.println(" RPM");

    
 
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
  }
  
}


// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}