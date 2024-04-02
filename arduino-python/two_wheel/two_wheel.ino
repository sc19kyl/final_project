#include <Arduino.h>
#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7


#define ENCC 11
#define ENCD 12
#define PWM2 8
#define IN3 9
#define IN4 10

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float receivedValue = 0;
float eintegral = 0;
float elast = 0;

int number1 = 0;
int number2 = 0;

// globals
long prevT2 = 0;
int posPrev2 = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i2 = 0;
volatile float velocity_i2 = 0;
volatile long prevT_i2 = 0;

float v1Filt2 = 0;
float v1Prev2 = 0;
float v2Filt2 = 0;
float v2Prev2 = 0;
float receivedValue2 = 0;
float eintegral2 = 0;
float elast2 = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  pinMode(ENCC,INPUT);
  pinMode(ENCD,INPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);

  attachInterrupt(digitalPinToInterrupt(ENCC),
                  readEncoder2,RISING);
}

void loop() {
  // Check if data is available to read

  if (Serial.available() > 0) {
    // Read the incoming message until newline character
    String message = Serial.readStringUntil('\n');
    
    // Print the received message for debugging
    Serial.println("Received message: " + message);
    
    // Check if the message is in the expected format
    if (message.indexOf(',') != -1) { // Ensure there's a comma in the message
      // Parse the message to extract integers
      number1 = message.substring(0, message.indexOf(',')).toInt();
      number2 = message.substring(message.indexOf(',') + 1).toInt();
      
    } else {
      Serial.println("Invalid message format!");
    }
  }
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }
  
  int pos2 = 0;
  float velocity4 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos2 = pos_i2;
    velocity4 = velocity_i2;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  long currT2 = micros();
  float deltaT2 = ((float) (currT2-prevT2))/1.0e6;
  float velocity3 = (pos2 - posPrev2)/deltaT2;
  posPrev2 = pos2;
  prevT2 = currT2;

  // Convert count/s to RPM
  float v1 = velocity1/600.0*60.0;
  float v2 = velocity2/600.0*60.0;

  float v3 = velocity3/600.0*60.0;
  float v4 = velocity4/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  v1Filt2 = 0.854*v1Filt2 + 0.0728*v3 + 0.0728*v1Prev2;
  v1Prev2 = v3;
  v2Filt2 = 0.854*v2Filt2 + 0.0728*v4 + 0.0728*v2Prev2;
  v2Prev2 = v4;

  
  // Set a target
  float vt = number1;
  float vt2 = number2;
  // Compute the control signal u
  float kp = 13;
  float ki = 10;

  float e = vt-v1Filt ;

  float kp2 = 14;
  float ki2 = 10;

  float e2 = vt2-v1Filt2 ;

  eintegral = eintegral + e*deltaT;
  eintegral2 = eintegral2 + e2*deltaT2;

  float u = kp * e + ki * eintegral;
  float u2 = kp2 * e2 + ki2 * eintegral2;
  // Set the motor speed and direction
  int dir = 1;
  int pwr = (int) fabs(u);
  int pwr2 = (int) fabs(u2);
  if(pwr > 255){
    pwr = 255;
  }
  if(pwr < 255){
    pwr = 0;
  }

  if(pwr2 > 255){
    pwr2 = 255;
  }
  if(pwr2 < 255){
    pwr2 = 0;
  }
  if (vt == 0){
    pwr = 0;
    pwr2 = 0;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);
  setMotor(dir,pwr2,PWM2,IN3,IN4);
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(" ");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt2);
  Serial.print(" ");
  Serial.print(u);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}


void readEncoder2(){
  // Read encoder B when ENCA rises
  int b2 = digitalRead(ENCD);
  int increment2 = 0;
  if(b2>0){
    // If B is high, increment forward
    increment2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment2 = -1;
  }
  pos_i2 = pos_i2 + increment2;

  // Compute velocity with method 2
  long currT2 = micros();
  float deltaT2 = ((float) (currT2 - prevT_i2))/1.0e6;
  velocity_i2 = increment2/deltaT2;
  prevT_i2 = currT2;
}

