#include <Arduino.h>
#include <util/atomic.h>
#include <PID_v1.h>
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

float number1 = 0;
float number2 = 0;

int dir = 0;
int dir2 = 0;
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

float circumference = 2 * 3.14 * 30;

double input, output, setpoint;
double kp = 3;
double ki = 5;
double kd = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double input2, output2, setpoint2;
double kp2 = 3;
double ki2 = 5;
double kd2 = 0;
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);

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

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1); // Set PID update time (in ms)
  myPID.SetOutputLimits(0, 255); // Set output limits2

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(1); // Set PID update time (in ms)
  myPID2.SetOutputLimits(0, 255); // Set output limits

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
      number1 = message.substring(0, message.indexOf(',')).toFloat();
      number2 = message.substring(message.indexOf(',') + 1).toFloat();
      
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
  float v1 = velocity1/620.0*60.0;
  v1 = v1*0.10471975512*60;

  float v2 = velocity2/620.0*60.0;
  v2 = v2*0.10471975512*60;

  float v3 = velocity3/620.0*60.0;
  v3 = v3*0.10471975512*60;

  float v4 = velocity4/620.0*60.0;
  v4 = v4*0.10471975512*60;

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
  if (vt < 0){
    dir = -1;
  }
  if (vt > 0){
    dir = 1;
  }
  if (vt2 < 0){
    dir2 = -1;
  }
  if (vt2 > 0){
    dir2 = 1;
  }


  setpoint = abs(vt);
  input = abs(v1Filt);
  myPID.Compute();
  int pwr = abs(output);

  setpoint2 = abs(vt2);
  input2 = abs(v1Filt2);
  myPID2.Compute();
  int pwr2 = abs(output2);

  if (vt == 0){
    pwr = 0;
    pwr2 = 0;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);
  setMotor(dir2,pwr2,PWM2,IN3,IN4);
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.print(vt2);
  Serial.print(" ");
  Serial.print(v1Filt2);
  Serial.print(" ");
  Serial.print(pwr2);
  Serial.print(" ");
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

