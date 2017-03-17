#define InA1            8                       // INA motor pin
#define InB1            9                       // INB motor pin 
#define PWM1            10                      // PWM motor pin
#define InA2            13                       // INA motor pin
#define InB2            12                       // INB motor pin 
#define PWM2            11                      // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      4                       // encoder B pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      5                       // encoder B pin
#define LOOPTIME        10                     // PID loop time
#define PRINTTIME       5                     // Display result loop time
#define motorlimit 120

int PWM_val1 = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
int PWM_valR = 0;  
int PWM_valS1 = 60;
int PWM_valS2 = 60;
int setpoint = 7;      
int setpointR = 10;
int setpointS = 2;                             
static int count1 = 0;                             
static int count2 = 0;
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing


int setpoint1 = 107;

int errorF = 0;
int previousF = 0;
       
float Kp =    .56;                                // PID proportional control Gain
float Kd =    20;                                // PID Derivitave control gain

float Kp2 =    .5;                                // PID proportional control Gain
float Kd2 =    23;

float KpR =    0.5;                                // PID proportional control Gain
float KdR =    23;

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
 return constrain(command + int(pidTerm), 0, 255);
}        

int rotSpdPid2(int command, int targetValue, int currentValue)   {             // compute PWM value
float pidTerm = 0;                                                            // PID correction
int error = 0;                                  
static int last_errorS=0;                             
 error =   abs(targetValue) - abs(currentValue); 
 pidTerm = (KpS2 * error) + (KdS2 * (error - last_errorS));                            
 last_errorS = error;
 return constrain(command + int(pidTerm), 0, 255);
} 

//int rangePid(int command, int targetValue, int currentValue)   {             // compute PWM value
//float pidTerm = 0;                                                            // PID correction
//int error = 0;                                  
//static int last_errorR=0;                             
// error =  abs(currentValue)-abs(targetValue); 
// pidTerm = (KpR * error) + (KdR * (error - last_errorR));                            
// last_errorR = error;
// return constrain(command + int(pidTerm), 0, 170);
//}  
          
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
          
void motor(bool dirL, int spdL, bool dirR, int spdR){                    // 0= FRONT 1=BACK
  moveMotorL(spdL,dirL);                                                 // motor(0,100,0,100);
  moveMotorR(spdR,dirR);
}
void stop(){                    // 0= FRONT 1=BACK
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
   Serial.print(rotSpd(0));  Serial.print('\t');
   Serial.print("  PWM 1:");          
   Serial.print(PWM_valS1);  Serial.print('\t');
   
   Serial.print("  COUNT 2:");          
   Serial.print(rotSpd(1));  Serial.print('\t');
   Serial.print("  PWM 2:");          
   Serial.println(PWM_valS2); 
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
 motor(0,60,0,60);
 delay(500);
}

void loop() {
  
if ((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
    PWM_valS1 = rotSpdPid1(PWM_valS1,setpointS,rotSpd(0));
    PWM_valS2 = rotSpdPid2(PWM_valS2,setpointS,rotSpd(1));
    printMotorInfo();
    refreshCount();
    motor (0, PWM_valS1, 0, PWM_valS2);

}

}



