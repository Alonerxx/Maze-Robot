#include <SharpIR.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define ir1 A2
#define model 1080
SharpIR irfr(ir1, 25, 93, model);

#define ir2 A3
#define model 1080
SharpIR irfl(ir2, 25, 93, model);

#define ir3 A6
#define model 1080
SharpIR irbl(ir3, 25, 93, model);

#define ir4 A7
#define model 1080
SharpIR irbr(ir4, 25, 93, model);

#define PWML  5
#define PWMR  6
#define InAR  7
#define InBR  8
#define InAL  10
#define InBL  9
#define encodPinAL      3                       // encoder A pin
#define encodPinBL      11                       // encoder B pin
#define encodPinAR      2                       // encoder A pin
#define encodPinBR      4
#define LOOPTIME        5                     // PID loop time
#define PRINTTIME       5  
#define motorlimit      120

static int count1 = 0;                             
static int count2 = 0;
int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
int setpointA = 0;  
int setpointS = 20;
float Kp =    20;                                // PID proportional control Gain
float Kd =    0;   

float KpT =    13;                                // PID proportional control Gain
float KdT =    20;                                // PID Derivitave control gain
float KpD =    13;                                // PID proportional control Gain
float KdD =    20;                                // PID Derivitave control gain

int setpointcalibration = 0;
float KpC = 0.5;
float KdC = 20;
float KpS1 =    1;                                // PID proportional control Gain
float KdS1 =    0;
float KpS2 =    1;                                // PID proportional control Gain
float KdS2 =    0;

double rotSpd (bool side) {                         // 0=LEFT   1=RIGHT
  if (side == 0){
    return abs(count1);
  }
  else if (side == 1){
  return abs(count2);
  }
}

int disfl = 0;
int disfr = 0;
int disbl = 0;
int disbr = 0;
int errora = 0;
int disf = 0;
int disb = 0;
int disl = 0;
int disr = 0;

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;    

void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
          // if (PINB & 0b00000001)    count++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
          // else                      count--;                // if (digitalRead(encodPinB1)==LOW) 
            if(digitalRead(encodPinBL)==HIGH)   count1 --;
            else if (digitalRead(encodPinBL)==LOW)count1 ++;
            //Serial.println(count);
          }   
          
void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
          // if (PINB & 0b00000001)    count++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
          // else                      count--;                // if (digitalRead(encodPinB1)==LOW) 
            if(digitalRead(encodPinBR)==HIGH)   count2 --;
            else if (digitalRead(encodPinBR)==LOW)count2 ++;
            //Serial.println(count);
          }

void stop(){                                              
  motor(0,0,0,0);
}

void refreshCount(){
  count1=0;
  count2=0;
}

int rotSpdPid1(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error = 0;                                  
static int last_errorS=0;                             
 error =   abs(targetValue) - abs(currentValue); 
 pidTerm = (KpS1 * error) + (KdS1 * (error - last_errorS));                            
 last_errorS = error;
 return constrain(command + int(pidTerm), 0, 140);
}        

int rotSpdPid2(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error = 0;                                  
static int last_errorS=0;                             
 error =   abs(targetValue) - abs(currentValue); 
 pidTerm = (KpS2 * error) + (KdS2 * (error - last_errorS));                            
 last_errorS = error;
 return constrain(command + int(pidTerm), 0, 140);
} 

void moveMotorL(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(InAL,HIGH);
    digitalWrite(InBL,LOW);
  }
  else{    
    digitalWrite(InBL,HIGH);
    digitalWrite(InAL,LOW);
  }
  analogWrite(PWML,pwm);
  //Serial.println("drive left");
}

void moveMotorR(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(InAR,HIGH);
    digitalWrite(InBR,LOW);
  }
  else{    
    digitalWrite(InBR,HIGH);
    digitalWrite(InAR,LOW);
  }
  analogWrite(PWMR,pwm);
  //Serial.println("drive left");
}

void motor(bool dirL, int spdL, bool dirR, int spdR){  // 0= FRONT 1=BACK
  moveMotorL(spdL,dirL);                                                 // motor(0,100,0,100);
  moveMotorR(spdR,dirR);
}

void setup() {
  Wire.begin();
  Serial.begin(250000);

pinMode(PWML, OUTPUT); 
pinMode(InAL, OUTPUT);
pinMode(InBL, OUTPUT);
pinMode(PWMR, OUTPUT);
pinMode(InAR, OUTPUT);
pinMode(InBR, OUTPUT);
pinMode(encodPinAL, INPUT);
pinMode(encodPinBL, INPUT);
pinMode(encodPinAR, INPUT);
pinMode(encodPinBR, INPUT);
attachInterrupt(digitalPinToInterrupt(encodPinAL), rencoder1, RISING); 
 //external disturbance will interrupt the system. 
attachInterrupt(digitalPinToInterrupt(encodPinAR), rencoder2, RISING);
//pinMode (irfl, INPUT);
//pinMode (irfr, INPUT);
pinMode (ir1, INPUT);
pinMode (ir2, INPUT);
pinMode (ir3, INPUT);
pinMode (ir4, INPUT);
}

void loop() {
int disfl = irfl.distance();
int disfr = irfr.distance();
int disbl = irbl.distance();
int disbr = irbr.distance();
 
if ((millis()-lastMilli) >= LOOPTIME) {
  lastMilli = millis();
//  errora = ( ( abs( disfl - disfr ) + abs( disbl - disbr ) ) / 2);
//  disf = disfl - disfr;
//  disb = disbl - disbr;
//  disl = disfl - disbl;
//  disr = disfr - disbr;
//  Serial.print(errora);

    float tilt_error= disbr-disfr;
    float dis_right= (disfr + disbr)/2;
//    Serial.print(tilt_error);Serial.print('\t');  
//    Serial.print(dis_right);Serial.print('\t');
    tilt_error= tilt_error_filter(tilt_error);                    //acceptable error = (-6,6) ~~+- 15degree
    dis_right= dis_right_filter(dis_right);                       //dis =  (9,14)
    Serial.print(tilt_error);Serial.print('\t');  
    Serial.print(dis_right);Serial.print('\t');

    int base_pwm=75;
    bool left_dir;
    bool right_dir;
    int left_pwm=0;
    int right_pwm=0;
    bool tilt_flag=true;
    bool dis_right_flag=true;    
    int tilt_corr;               //range=(-base_speed,0)  eg : (-80,0)
    int dis_right_corr;           //range=(-base_speed,0)
    
    #define tilt_deadband 1 
    #define tilt_setpoint 0
    #define dis_right_deadband 1   
    #define dis_right_setpoint 11.5  //maybe 11.5
    
    if ( abs(tilt_error-tilt_setpoint) > tilt_deadband ){
        tilt_corr = tilt_pid(tilt_setpoint,tilt_error);
        tilt_flag = false;
    }
    else tilt_flag=true;
    if ( abs(dis_right-dis_right_setpoint) > dis_right_deadband ){
        dis_right_corr = dis_right_pid(dis_right_setpoint,dis_right);
        dis_right_flag = false; 
    }
    else dis_right_flag=true; 
    
    Serial.print( tilt_corr);Serial.print('\t');  
    Serial.print(dis_right_corr);Serial.print('\t');
    
    float alpha = 1.13;  //range=(1,2)   eg : 1.2 ~ 60%   1.4~70%
    
    left_pwm =  base_pwm - (alpha) * tilt_corr + (2 - alpha) * dis_right_corr;  
    right_pwm = base_pwm + (alpha) * tilt_corr - (2 - alpha) * dis_right_corr;
    left_pwm = constrain(left_pwm, -255, 100);
    right_pwm = constrain(right_pwm, -255, 80);
    
    if (left_pwm>0)left_dir=0;               //0 is forward 1 is reverse
    else if(left_pwm<0)left_dir=1;
    if (right_pwm>0)right_dir=0;
    else if(right_pwm<0)right_dir=1;
    
  motor (left_dir, abs(left_pwm), right_dir, abs(right_pwm));
    Serial.print(left_pwm); Serial.print('\t');  
    Serial.print(right_pwm); Serial.print('\t');  
    Serial.println();

//    if (( tilt_flag==true)&&(dis_right_flag==true)){
//        left_pwm=base_pwm;
//        right_pwm=left_pwm;
//        motor (left_dir, abs(left_pwm), right_dir, abs(right_pwm));
//    }
//    else{
//        motor (left_dir, abs(left_pwm), right_dir, abs(right_pwm));
//    }

}

}

int tilt_pid(int targetError, int currentError){   
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error =  currentError-targetError;
pidTerm = (KpT * error) + (KdT * (error - last_error));                              
last_error = error;
return constrain(int(pidTerm),-100, 100);
}

int dis_right_pid(int targetError, int currentError){    
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error = abs (currentError) - abs(targetError);
pidTerm = (KpD * error) + (KdD * (error - last_error));                              
last_error = error;
return constrain(int(pidTerm),-100, 100);
}

float dis_right_filter(float current_value){
  static float last_value=0;
  static float last_last_value=0;
  float ans=(current_value+last_value+last_last_value)/3;  
  last_last_value=last_value;
  last_value=current_value;
  return ans;
}

float tilt_error_filter(float current_value){
  static float last_value=0;
  static float last_last_value=0;
  float ans=(current_value+last_value+last_last_value)/3;  
  last_last_value=last_value;
  last_value=current_value;
  return ans;
}

void printIR(){
      Serial.print("fl  ");
      Serial.print(disfl);
      Serial.print('\t');
      Serial.print("fr  ");
      Serial.print(disfr);
      Serial.println('\t');
      Serial.print("bl  ");
      Serial.print(disbl);
      Serial.print('\t');
      Serial.print("br  ");
      Serial.print(disbr);
      Serial.print('\t');  
      Serial.println();
}

void computeError(){
  //  if (errora > 1) {
//    if ( (disf > 1) && (disb < -1) ) {
//      //turn left
//      PWM_val1 = irpid2(PWM_val1, 0, disf);
//      PWM_val2 = 70;
//    } else if ( (disf < -1) && (disb > 1) ) {
//      //turn right
//      PWM_val1 = (irpid(PWM_val1, 0, disf)) - 5;
//      PWM_val2 = 70;
//    } else if ( (disf > 1) && (disb > 1) ) {
//      // turn right
//      PWM_val1 = (irpid(PWM_val1, 0, disf)) - 5;
//      PWM_val2 = 70;
//    } else if ( (disf < -1) && (disl < -1) ) {
//      //turn left
//      PWM_val1 = irpid2(PWM_val1, 0, disf);
//      PWM_val2 = 70;
//    }
//  } else {
//     PWM_val1 = rotSpdPid1(PWM_val1,setpointS,count1);
//    PWM_val2 = rotSpdPid2(PWM_val2,setpointS,count2);
//    //printMotorInfo();
//    refreshCount();
//  }
}

int passivePid(int command, int targetAngle, int currentAngle)   {           // compute PWM value
float pidTerm = 0;                                                       // PID correction
int error = 0;                                                           // neg currentAngle = slanted right = slave too fast = error positive
static int last_errorPWM = 0;                             
 error =  abs(targetAngle) - abs(currentAngle); 
 pidTerm = (Kp * error) + (Kd * (error - last_errorPWM));               // calculates whether it slow down or speeds up                      
 last_errorPWM = error;
 if (currentAngle < 0) {
 return constrain(command + int(pidTerm), 45, 80);
 } else if (currentAngle > 0) {
  return constrain(command - int(pidTerm), 45, 80);
 }
}

int aggressivePid(int command, int targetAngle, int currentAngle)   {           // compute PWM value
float pidTerm = 0;                                                       // PID correction
int error = 0;                                                           // neg currentAngle = slanted right = slave too fast = error positive
static int last_errorPWM = 0;                             
 error =  abs(targetAngle) - abs(currentAngle); 
 pidTerm = (Kp * error) + (Kd * (error - last_errorPWM));               // calculates whether it slow down or speeds up                      
 last_errorPWM = error;
 if (currentAngle < 0) {
 return constrain(command + int(pidTerm), 0, 150);
 } else if (currentAngle > 0) {
  return constrain(command - int(pidTerm), 0, 150);
 }
}

int irpid(int command, int targetError, int currentError){    //turn right
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error = abs (currentError) - abs(targetError);
pidTerm = (Kp * error) + (Kd * (error - last_error));                              
last_error = error;
return constrain(command + int(pidTerm), 70, 140);
}

int irpid2(int command, int targetError, int currentError){   //turn left
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error = abs (currentError) - abs(targetError);
pidTerm = (KpC * error) + (KdC * (error - last_error));                              
last_error = error;
return constrain(command - int(pidTerm), 20, 70);
}

