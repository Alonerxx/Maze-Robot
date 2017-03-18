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
int setpointA = 0;                     
static int count1 = 0;                             
static int count2 = 0;
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing


float Kp =    .56;                                // PID proportional control Gain
float Kd =    20;                                // PID Derivitave control gain

float Kp2 =    .5;                                // PID proportional control Gain
float Kd2 =    23;

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
 motor(0,70,0,0);

}

void loop() {
 
}
  
int PWM_RPid(int command, int targetAngle, int currentAngle)   {           // compute PWM value
float pidTerm = 0;                                                       // PID correction
int error = 0;                                                           // neg currentAngle = slanted right = slave too fast = error positive
static int last_errorPWM = 0;                             
 error =  abs(targetAngle) - abs(currentAngle); 
 pidTerm = (KpR * error) + (KdR * (error - last_errorR));               // calculates whether it slow down or speeds up                      
 last_errorR = error;
 if (currentAngle < 0) {
 return constrain(command - int(pidTerm), 0, 150);
 } else if (currentAngle > 0) {
  return constrain(command + int(pidTerm), 0, 150);
 }

void motor(bool dirL, int spdL, bool dirR, int spdR){                    // 0 = FRONT 1 = BACK
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



