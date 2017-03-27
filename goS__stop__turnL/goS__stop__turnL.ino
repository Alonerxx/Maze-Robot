#include <SharpIR.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

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

#define ir5 A0
#define model 1080
SharpIR irf1(ir5, 25, 93, model);

#define ir6 A1
#define model 1080
SharpIR irf2(ir6, 25, 93, model);

#define PWML  5
#define PWMR  6
#define InAR  7
#define InBR  8
#define InAL  10
#define InBL  9
#define encodPinAL      3                       // encoder A pin
#define encodPinBL      11                      // encoder B pin
#define encodPinAR      2                       // encoder A pin
#define encodPinBR      4
#define LOOPTIME        5                       // PID loop time
#define PRINTTIME       5                      // Display result loop time

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
MPU6050 mpu;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

float KpT =    13;                                // PID proportional control Gain
float KdT =    40;                                // PID Derivitave control gain
float KpD =    13;                                // PID proportional control Gain
float KdD =    40;                                // PID Derivitave control gain
float KpF =    10; 
float KdF =    150; 
float KpR =    10;                                // PID proportional control Gain
float KdR =    150; 
float Kp =    1;                                // PID proportional control Gain
float Kd =    20;                                // PID Derivitave control gain

int disfl = 0;
int disfr = 0;
int disbl = 0;
int disbr = 0;
int disf1 = 0;
int disf2 = 0;
int setpointA = 45;  
int left_pwm=60;
int right_pwm=60; 

bool turnFlag = false;


// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;


void calibrate_sensors() {
  int       num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}
static int count1 = 0;                             
static int count2 = 0;

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


int TURNPid(int command, int targetAngle, int currentAngle)   {           // compute PWM value
float pidTerm = 0;                                                       // PID correction
int error = 0;                                                           // neg currentAngle = slanted right = slave too fast = error positive
static int last_errorPWM = 0;                             
 error =  abs(targetAngle) - abs(currentAngle); 
 pidTerm = (Kp * error) + (Kd * (error - last_errorPWM));               // calculates whether it slow down or speeds up                      
 last_errorPWM = error;
 return constrain(command + int(pidTerm), 0, 100);
}


void stop(){                                              
  motor(0,0,0,0);
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
  //Serial.println("drive right");
}

void motor(bool dirL, int spdL, bool dirR, int spdR){  // 0= FRONT 1=BACK
  moveMotorL(spdL,dirL);                                                 // motor(0,100,0,100);
  moveMotorR(spdR,dirR);
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

float tilt_error_filter(float current_value){
  static float last_value=0;
  static float last_last_value=0;
  float ans=(current_value+last_value+last_last_value)/3;  
  last_last_value=last_value;
  last_value=current_value;
  return ans;
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

int dis_front_pid1(int targetDis, int currentDis){    
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error = (currentDis) - (targetDis);
pidTerm = (KpF * error) + (KdF * (error - last_error));                              
last_error = error;
return constrain (float(pidTerm), 60, 70);
}

int dis_front_pid2(int targetDis, int currentDis){    
float pidTerm = 0;
int error = 0;
static int last_error = 0;
error = (currentDis) - (targetDis);
pidTerm = (KpR * error) + (KdR * (error - last_error));                              
last_error = error;
return constrain (float(pidTerm), -60, -70);
}

float dis_front_filter(float current_value) {
  static float last_value = 0;
  static float last_last_value = 0;
  float ans = (current_value + last_value + last_last_value)/3;  
  last_last_value = last_value;
  last_value = current_value;
  return ans;
}

void computeMPU(){
  const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
  unsigned long t_now = millis();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = ax; // - base_x_accel;
        float accel_y = ay; // - base_y_accel;
        float accel_z = az; // - base_z_accel;

        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        float dt =(t_now - get_last_time())/1000.0;
        float gyro_angle_x = gyro_x*dt + get_last_x_angle();
        float gyro_angle_y = gyro_y*dt + get_last_y_angle();
        float gyro_angle_z = gyro_z*dt + get_last_z_angle();
        
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
        float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
        float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();     


        // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.96;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
        float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
        
//        if (abs(angle_z)-abs(get_last_z_angle())<0.05)
//        angle_z=get_last_z_angle();
//        Serial.println(angle_z);
        // Update the saved data with the latest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
}


void goS (){
        int disfr = irfr.distance();
        int disbr = irbr.distance();
        int base_pwm=75;
        int left_pwm=0;
        int right_pwm=0;  
        bool left_dir;
        bool right_dir;
        int tilt_corr;               
        int dis_right_corr;           
        float tilt_error= disbr-disfr;
        float dis_right= (disfr + disbr)/2;
        tilt_error= tilt_error_filter(tilt_error);                    
        dis_right= dis_right_filter(dis_right);                    
        
        #define tilt_deadband 1 
        #define tilt_setpoint 0
        #define dis_right_deadband 1   
        #define dis_right_setpoint 11.5  
        
                Serial.print (tilt_error); Serial.print('\t');
                Serial.print (dis_right); Serial.print ('\t');
                
                if ( abs(tilt_error-tilt_setpoint) > tilt_deadband ){
                    tilt_corr = tilt_pid(tilt_setpoint,tilt_error);    
                }
             
                if ( abs(dis_right-dis_right_setpoint) > dis_right_deadband ){
                    dis_right_corr = dis_right_pid(dis_right_setpoint,dis_right);                   
                }
                
                Serial.print (tilt_corr); Serial.print('\t');
                Serial.print (dis_right_corr); Serial.print ('\t');
                
                float alpha = 1.13;
                left_pwm =  base_pwm - (alpha) * tilt_corr + (2 - alpha) * dis_right_corr;  
                right_pwm = base_pwm + (alpha) * tilt_corr - (2 - alpha) * dis_right_corr;
                left_pwm = constrain(left_pwm, -255, 80);
                right_pwm = constrain(right_pwm, -255, 100);

                Serial.print (left_pwm); Serial.print ('\t');
                Serial.println (right_pwm);
                
                if (left_pwm>0)left_dir=0;               //0 is forward 1 is reverse
                else if(left_pwm<0)left_dir=1;
                if (right_pwm>0)right_dir=0;
                else if(right_pwm<0)right_dir=1;
                
                motor (left_dir, abs(left_pwm), right_dir, abs(right_pwm));

}

bool calibrate () {
        int disf1 = irf1.distance();
        int disf2 = irf2.distance();
        int disfr = irfr.distance();
        int disbr = irbr.distance();
        int base_pwm = 0;
        int left_pwm = 0;
        int right_pwm = 0;  
        bool left_dir;
        bool right_dir;
        int dis_f1_corr;            
        int dis_f2_corr;
        int tilt_corr;
        disf1 = dis_front_filter(disf1);                           
        disf2 = dis_front_filter(disf2);  
        float tilt_error = disbr - disfr;
        tilt_error = tilt_error_filter(tilt_error);  
        
        #define tilt_deadband 1 
        #define tilt_setpoint 0       
        #define dis_front_setpoint 10
 
        
        if ( (disf1 == dis_front_setpoint ) && (disf2 == dis_front_setpoint) ) {
            if (tilt_error >1){
                dis_f1_corr = dis_front_pid2(tilt_setpoint, tilt_error);
            }
            else if (tilt_error <-1) {
                dis_f2_corr = dis_front_pid2(tilt_setpoint, tilt_error); 
            }
            else {
                dis_f1_corr = 0;
                dis_f2_corr = 0;
            }
        }
        
        if ( (disf1 == dis_front_setpoint) && (disf2 > dis_front_setpoint) ) {
          dis_f1_corr = 0;
          dis_f2_corr = dis_front_pid1(dis_front_setpoint, disf2);
        } 
        
        if ( (disf2 == dis_front_setpoint) && (disf1 > dis_front_setpoint) ) {
          dis_f2_corr = 0;
          dis_f1_corr = dis_front_pid1(dis_front_setpoint, disf1);
        } 

        if ( (disf1 == dis_front_setpoint) && (disf2 < dis_front_setpoint) ) {
          dis_f1_corr = 0;
          dis_f2_corr = dis_front_pid2(dis_front_setpoint, disf2);
        } 
        
        if ( (disf2 == dis_front_setpoint) && (disf1 < dis_front_setpoint) ) {
          dis_f2_corr = 0;
          dis_f1_corr = dis_front_pid2(dis_front_setpoint, disf1);
        } 

        if ( (disf1 < dis_front_setpoint) && (disf2 < dis_front_setpoint) ) {
          dis_f1_corr = dis_front_pid2(dis_front_setpoint, disf1);;
          dis_f2_corr = dis_front_pid2(dis_front_setpoint, disf2);
        } 

        if ( (disf1 > dis_front_setpoint) && (disf2 > dis_front_setpoint) ) {
          dis_f1_corr = dis_front_pid1(dis_front_setpoint, disf1);;
          dis_f2_corr = dis_front_pid1(dis_front_setpoint, disf2);
        }
                  
        left_pwm = dis_f1_corr;  
        right_pwm = dis_f2_corr;
        left_pwm = constrain(left_pwm, -100, 100);
        right_pwm = constrain(right_pwm, -100, 80);
        
        if (left_pwm > 0) left_dir = 0;               //0 is forward 1 is reverse
        else if(left_pwm < 0) left_dir = 1;
        if (right_pwm > 0) right_dir = 0;
        else if(right_pwm < 0) right_dir = 1;
        
        Serial.print (disf1); Serial.print ('\t');
        Serial.print (disf2); Serial.print ('\t');
        Serial.print (tilt_error); Serial.print ('\t');
        Serial.print (left_pwm); Serial.print ('\t');
        Serial.println (right_pwm);
      motor (left_dir, abs(left_pwm), right_dir, abs(right_pwm));

      if ( (dis_f1_corr==0) && (dis_f2_corr==0) ) {
        return true;
      }
      else return false;

}

void turnL(){
 
    
    stop();//delay(200);    
    Serial.println ("Hello");
    if( (-get_last_z_angle()<setpointA)){
         left_pwm = TURNPid(left_pwm,setpointA,-get_last_z_angle());
    }
    right_pwm=left_pwm;
    motor(1,left_pwm,0,right_pwm);
    //printInfo();
     Serial.println ("wtf"); 
      stop();
      turnFlag=false;

}

 void printInfo(){         // display data
 if((millis()-lastMilliPrint) >= PRINTTIME)   {                     
   lastMilliPrint = millis();
//   Serial.print("SP:");             
//   Serial.print(speed_req);  Serial.print('\t');
   Serial.print("  COUNT 1:");          
   Serial.print(count1);  Serial.print('\t');
   Serial.print("  Left PWM:");          
   Serial.print(left_pwm);  Serial.print('\t');
   
   Serial.print("  COUNT 2:");          
   Serial.print(count2);  Serial.print('\t');
   Serial.print("  Right PWM:");          
   Serial.print(right_pwm); Serial.print('\t');
 }
 printMPU();Serial.println();
}

 void printMPU(){
       Serial.print("CMP:");
       Serial.print(get_last_x_angle(), 2);
       Serial.print(":");
       Serial.print(get_last_y_angle(), 2);
       Serial.print(":");
       Serial.print(-get_last_z_angle(), 2);
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
  attachInterrupt(digitalPinToInterrupt(encodPinAR), rencoder2, RISING);
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  pinMode (ir4, INPUT);
  pinMode (ir5, INPUT);
  pinMode (ir6, INPUT);

  while (!Serial);
  
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  uint8_t FS_SEL = 0;
  uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
          Serial.print("FS_SEL = ");
          Serial.println(READ_FS_SEL);
          GYRO_FACTOR = 131.0/(FS_SEL + 1);  
  
  uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
          Serial.print("AFS_SEL = ");
          Serial.println(READ_AFS_SEL);
  
      calibrate_sensors();
      set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}


void loop() {
int disfl = irfl.distance();
int disfr = irfr.distance();
int disbl = irbl.distance();
int disbr = irbr.distance();
int disf1 = irf1.distance();
int disf2 = irf2.distance();


  if ((millis()-lastMilli) >= LOOPTIME) {
      lastMilli = millis();  

      disf1 = dis_front_filter(disf1);                       
      disf2 = dis_front_filter(disf2); 
      float dis_front = (disf1 + disf2)/2; 

//        Serial.print (disf1); Serial.print ('\t');
//        Serial.print (disf2); Serial.print ('\t');
//        Serial.print (left_pwm); Serial.print ('\t');
//        Serial.println (right_pwm);
        
      if (dis_front <= 15) {

          if (calibrate() == true) {
            turnFlag = true;
          }
      }

      else if (dis_front>15){
           goS();                  
      }


       if (turnFlag==true) {
           Serial.println ("Ready to turn");
           printInfo();
           stop(); delay(2000);
           setpointA = (-get_last_z_angle()) + 90;
              while( (-get_last_z_angle()<setpointA)){
                  computeMPU();
                  printInfo(); 
                  left_pwm = TURNPid(left_pwm,setpointA,-get_last_z_angle());
                  right_pwm=left_pwm;
                  motor(1,left_pwm,0,right_pwm);
              }
            stop();
            turnFlag = false;                  
       }

  }
            
}



