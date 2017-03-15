#include <SharpIR.h>

#define ir_left A1
#define model 1080
SharpIR sharp_left(ir_left, 25, 93, model);

#define ir_right A2
#define model 1080
SharpIR sharp_right(ir_right, 25, 93, model);

// ir: the pin where your sensor is attached
// 25: the number of readings the library will make before calculating a mean distance
// 93: the difference between two consecMutive measurements to be taken as valid
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

#define InA1            8                       // INA motor pin
#define InB1            9                       // INB motor pin 
#define PWM1            10                      // PWM motor pin
#define InA2            12                       // INA motor pin
#define InB2            13                       // INB motor pin 
#define PWM2            11                      // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      4                       // encoder B pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      5                       // encoder B pin
#define LOOPTIME        5                     // PID loop time
#define PRINTTIME       5                     // Display result loop time


int PWM_val1 = 40;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 40;
int PWM_valR = 40;
int currentpoint1 = 0;                           
int currentpoint2 = 0;                      
int currentpointR = 0;   
int setpoint = 7;      
int setpointR = 12;                             
static int count1 = 0;                             
static int count2 = 0;
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing


float Kp =    .56;                                // PID proportional control Gain
float Kd =    20;                                // PID Derivitave control gain

float Kp2 =    .5;                                // PID proportional control Gain
float Kd2 =    23;

float KpR =    0.5;                                // PID proportional control Gain
float KdR =    23;


  
void setup() { 
 Serial.begin(250000);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
 pinMode(InA2, OUTPUT);
 pinMode(InB2, OUTPUT);
 pinMode(PWM2, OUTPUT);
 pinMode(encodPinA1, INPUT);
 pinMode(encodPinB1, INPUT);
 pinMode(encodPinA2, INPUT);
 pinMode(encodPinB2, INPUT);
 analogWrite(PWM1, 170);
 digitalWrite(InB1, HIGH);
 digitalWrite(InA1, LOW); 
 analogWrite(PWM2, 170);
 digitalWrite(InB2, HIGH);
 digitalWrite(InA2, LOW);

 pinMode (ir_left, INPUT);
 pinMode (ir_right, INPUT);
}

void loop() {
  int dis_left = sharp_left.distance();
  int dis_right = sharp_right.distance();// this returns the distance to the object you're measuring
  Serial.print(dis_left);
  Serial.print("\t");
  Serial.println (dis_right);
  int error = dis_right - dis_left;
  
  if (error > 1) {
    analogWrite(PWM1, 180);
    analogWrite(PWM2, 170);
    Serial.println("turn right");
  } else if (error < -1) {
    analogWrite(PWM1, 170);
    analogWrite(PWM2, 180);
    Serial.println("turn left");
  } else {
    analogWrite(PWM1, 170);
    analogWrite(PWM2, 170);
    Serial.println("straight");
    
  }
}  
    


