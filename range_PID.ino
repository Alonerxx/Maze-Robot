#include <SharpIR.h>

#define ir A0
#define model 1080
SharpIR sharp(ir, 10, 93, model);

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
// analogWrite(PWM1, 100);
 digitalWrite(InB1, HIGH);
 digitalWrite(InA1, LOW); 
// analogWrite(PWM2, 100);
 digitalWrite(InB2, HIGH);
 digitalWrite(InA2, LOW);
 
 pinMode (ir, INPUT);
}

void loop() {

  unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  int dis = sharp.distance();  // this returns the distance to the object you're measuring
  Serial.print(dis);
  Serial.print("\t");
  unsigned long pepe2=millis()-pepe1;  // the following gives you the time taken to get the measurement
  //Serial.println(pepe2);  
  
  if((millis()-lastMilli) >= LOOPTIME)   { 
    lastMilli = millis(); 
//    PWM_val1= updatePid1(PWM_val1, setpoint, dis);                        // compute PWM value
//    PWM_val2= updatePid2(PWM_val2, setpoint, dis);                        // compute PWM value
//    if ( count1<setpoint){
//       digitalWrite(InB1, HIGH);
//       digitalWrite(InA1, LOW);
//    }
//    else {
//       digitalWrite(InB1, LOW);
//       digitalWrite(InA1, HIGH);
//    }
//
//    if ( count2<setpoint){
//       digitalWrite(InB2, HIGH);
//       digitalWrite(InA2, LOW);
//    }
//    else {
//       digitalWrite(InB2, LOW);
//       digitalWrite(InA2, HIGH);
//    }
    //if (dis>setpointR)
    PWM_valR=rangePid(PWM_valR,setpointR,dis);
    analogWrite(PWM1, PWM_valR);                                               // send PWM to motor                      
    analogWrite(PWM2, PWM_valR);                                               // send PWM to motor                      
    Serial.println(PWM_valR);
  }
}

int rangePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error = 0;                                  
static int last_errorR=0;                             
 error =  abs(currentValue)-abs(targetValue); 
 pidTerm = (KpR * error) + (KdR * (error - last_errorR));                            
 last_errorR = error;
 return constrain(command + int(pidTerm), 0, 170);
}

