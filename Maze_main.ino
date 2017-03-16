#include <SharpIR.h>

#define ir_front A2
#define model 1080
SharpIR sharp_front(ir_front, 25, 93, model);

#define ir_left A1
#define model 1080
SharpIR sharp_left(ir_left, 25, 93, model);

#define ir_right A0
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
#define motorlimit 120
int motorPin[6]={InA1,InB1,PWM1,InA2,InB2,PWM2};

int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
int PWM_valR = 0;
int setpoint = 7;
int setpointR= 12;                             
static int count1 = 0;                             
static int count2 = 0;
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing

int setpointT = 107;
       
float Kp =    .56;                                // PID proportional control Gain
float Kd =    20;                                // PID Derivitave control gain

float Kp2 =    .5;                                // PID proportional control Gain
float Kd2 =    23;

float KpR =    0.5;                                // PID proportional control Gain
float KdR =    23;

bool turnFlag = false;

bool calibrate(){
//  coding here
  
}

void turnL(){
    stop();delay(200);    
    count1=0;count2=0;
    Serial.println("Ready to turn");
    while((-count1<setpointT)|| (count2<setpointT)){
          PWM_val1= updatePid1(PWM_val1, setpointT, count1);                        // compute PWM value
          PWM_val2= updatePid2(PWM_val2, setpointT, count2);                        // compute PWM value    
          motor(1,PWM_val1,0,PWM_val2);
          printMotorInfo();
      }
      stop();
      turnFlag=false;
      Serial.println("Stop");
      delay(1000);
}

int updatePid1(int command, int targetValue, int currentValue)   {             // compute PWM value
          float pidTerm = 0;                                                            // PID correction
          int error=0;                                  
          static int last_error1=0;                             
           error = abs(targetValue) - abs(currentValue); 
           pidTerm = (Kp * error) + (Kd * (error - last_error1));                            
           last_error1 = error;
           return constrain(command + int(pidTerm), 0, motorlimit);
          }
          
int updatePid2(int command, int targetValue, int currentValue)   {             // compute PWM value
          float pidTerm = 0;                                                            // PID correction
          int error=0;                                  
          static int last_error2=0;                             
           error = abs(targetValue) - abs(currentValue); 
           pidTerm = (Kp2 * error) + (Kd2 * (error - last_error2));                            
           last_error2 = error;
           return constrain(command + int(pidTerm), 0, motorlimit);
          }
          
 void rencoder1()  {                                    // pulse and direction, direct port reading to save cycles
          // if (PINB & 0b00000001)    count++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
          // else                      count--;                // if (digitalRead(encodPinB1)==LOW) 
            if(digitalRead(encodPinB1)==HIGH)   count1 --;
            else if (digitalRead(encodPinB1)==LOW)count1 ++;
            //Serial.println(count);
          }
          
void rencoder2()  {                                    // pulse and direction, direct port reading to save cycles
          // if (PINB & 0b00000001)    count++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
          // else                      count--;                // if (digitalRead(encodPinB1)==LOW) 
            if(digitalRead(encodPinB2)==HIGH)   count2 --;
            else if (digitalRead(encodPinB2)==LOW)count2 ++;
            //Serial.println(count);
          }
          
int rangePid(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error = 0;                                  
static int last_errorR=0;                             
 error =  abs(currentValue)-abs(targetValue); 
 pidTerm = (KpR * error) + (KdR * (error - last_errorR));                            
 last_errorR = error;
 return constrain(command + int(pidTerm), 0, motorlimit);
}  

void motor(bool dirL, int spdL, bool dirR, int spdR){                    // 0= FRONT 1=BACK
  moveMotorL(spdL,dirL);                                                 // motor(0,100,0,100);
  moveMotorR(spdR,dirR);
}
void stop(){                    // 0 = FRONT  1 = BACK
  motor(0,0,0,0);
}
void moveMotorL(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(InA1,HIGH);
    digitalWrite(InB1,LOW);
  }
  else{    
    digitalWrite(InB1,HIGH);
    digitalWrite(InA1,LOW);
  }
  analogWrite(PWM1,pwm);
  //Serial.println("drive left");
}

void moveMotorR(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(InA2,HIGH);
    digitalWrite(InB2,LOW);
  }
  else{    
    digitalWrite(InB2,HIGH);
    digitalWrite(InA2,LOW);
  }
  analogWrite(PWM2,pwm);
  //Serial.println("drive left");
}


 void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= PRINTTIME)   {                     
   lastMilliPrint = millis();
//   Serial.print("SP:");             
//   Serial.print(speed_req);  Serial.print('\t');
   Serial.print("  COUNT 1:");          
   Serial.print(count1);  Serial.print('\t');
   Serial.print("  PWM 1:");          
   Serial.print(PWM_val1);  Serial.print('\t');
   
   Serial.print("  COUNT 2:");          
   Serial.print(count2);  Serial.print('\t');
   Serial.print("  PWM 2:");          
   Serial.println(PWM_val2); 
 }
}

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
 attachInterrupt(digitalPinToInterrupt(encodPinA1), rencoder1, RISING);
 attachInterrupt(digitalPinToInterrupt(encodPinA2), rencoder2, RISING);
 pinMode (ir_front, INPUT);
 pinMode (ir_left, INPUT);
 pinMode (ir_right, INPUT);
 motor(0,60,0,60);
 delay(500);
}

void loop() {
  int dis = sharp_front.distance();
  dis = constrain(dis, 0, 80); 
  Serial.println (dis);

  if ((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
        if (dis > setpointR + 1) {                              //front sensor
           motor(0,60,0,60);
           } 
        else if (dis < setpointR - 1) {
           Serial.println("TURRRRRRRRNNNNNNNNNNNNN");          //VALID TURN?
           turnFlag = true; 
           }
        if (turnFlag == true){
           while(calibrate() == false){};         
           turnL();        
           }
  }
}
