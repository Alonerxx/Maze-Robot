#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
#define LOOPTIME        5                     // PID loop time
#define PRINTTIME       5                     // Display result loop time

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing
MPU6050 mpu;

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

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
          
void motor(bool dirL, int spdL, bool dirR, int spdR){  // 0= FRONT 1=BACK
  moveMotorL(spdL,dirL);                                                 // motor(0,100,0,100);
  moveMotorR(spdR,dirR);
}
void stop(){                                              
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

int PWM_val1 = 60;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 60;
int setpointA = 0;  
float Kp =    1;                                // PID proportional control Gain
float Kd =    30;                                // PID Derivitave control gain
               


 void printInfo(){         // display data
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
   Serial.print(PWM_val2); Serial.print('\t');
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
 void plotInfo(){                                                      
 if((millis()-lastMilliPrint) >= PRINTTIME)   {                     
   lastMilliPrint = millis();
//   Serial.print("SP:");             
//   Serial.print(speed_req);  Serial.print('\t');
   //Serial.print("  COUNT 1:");          
   Serial.print(count1);  Serial.print('\t');
   //Serial.print("  PWM 1:");          
   Serial.print(PWM_val1);  Serial.print('\t');
   
  // Serial.print("  COUNT 2:");          
   Serial.print(count2);  Serial.print('\t');
  // Serial.print("  PWM 2:");          
   Serial.println(PWM_val2); 
 }
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


void setup() {
  Wire.begin();
  Serial.begin(57600);
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
}

void loop() {
  computeMPU();
  
    if ((millis()-lastMilli) >= LOOPTIME) {
       lastMilli = millis();
       if( (-get_last_z_angle()<setpointA - 5) || (-get_last_z_angle()>setpointA + 5)) {
           PWM_val2 = aggressivePid(PWM_val2,setpointA,-get_last_z_angle());
           }
       else if( (-get_last_z_angle()<setpointA - 2) || (-get_last_z_angle()>setpointA + 2)){
           PWM_val2 = passivePid(PWM_val2,setpointA,-get_last_z_angle());
           }
       else if( (-get_last_z_angle()<setpointA - 0.5) || (-get_last_z_angle()>setpointA + 0.5)){
           PWM_val2=PWM_val1;
           }
       motor(0,PWM_val1,0,PWM_val2);
       printInfo();
      }       
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

