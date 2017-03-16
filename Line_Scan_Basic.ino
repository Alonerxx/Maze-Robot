#define leftDir1  8                //Motor Pin Assignment
#define leftDir2  7
#define leftspd   6
#define rightDir1 4
#define rightDir2 5
#define rightspd 3
int motorPin[6]={leftDir1,leftDir2,leftspd,rightDir1,rightDir2,rightspd};
int backpass=1;
#define irL A0
#define irR A1
#define black 0
#define white 1
int spd=75,tL=850,tR=400;

void setup() {

  for(int i=0;i<6;i++){
    pinMode(motorPin[i],OUTPUT);
  }
  pinMode(irL,INPUT);
  pinMode(irR,INPUT);
  Serial.begin(38400);
}

void loop() {
  /* //Serial Monitor Testing
  if(Serial.available()>0){
    char rx=Serial.read();
    if (rx!='\n'){
    char temp=rx;
    drive(temp);
    
    }
  }
  */
  //scanL();scanR();
  //Serial.print(scanL());
  //Serial.print('\t');
  //Serial.println(scanR());
  if ((scanR()==black && scanL()==white )||(scanR()==white && scanL()==black ))drive('f');
  else if (scanR()==white || scanL()==white)drive('l');
  //else if (scanL()==0)drive('l');
  else drive('b');
  //*/
}

bool scanR(){
  int a = analogRead(irR);
  Serial.print(a);
  Serial.print('\t');
  if (a>tR) return 1;
  else return 0;
}
bool scanL(){
  int a = analogRead(irL);
  Serial.println(a);
  if (a>tL) return 1;
  else return 0;
}

void drive(char a){
  switch(a){
    case 'b' :{
      if(backpass==1){
      moveMotorL(spd+40,1);
      moveMotorR(spd-40,0);
      delay(300);
      Serial.println("Front");
      }
      moveMotorL(0,1);
      moveMotorR(0,1);
      backpass++;
      break;
    }
    case 'l' :{
      moveMotorL(spd/3,1);
      moveMotorR(spd,0);
      Serial.println("Left");
      break;
    }
    case 'r' :{
      moveMotorL(spd,0);
      moveMotorR(spd,1);
      Serial.println("Right");
      break;
    }
    case 'f' :{
      moveMotorL(spd+50,0);
      moveMotorR(spd-40,0);
      Serial.println("Back");
      break;
    }
    case 's' :{
      Serial.println("Stop");
      break;
    }
  }
}

void moveMotorL(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(leftDir1,HIGH);
    digitalWrite(leftDir2,LOW);
  }
  else{    
    digitalWrite(leftDir2,HIGH);
    digitalWrite(leftDir1,LOW);
  }
  analogWrite(leftspd,pwm);
  //Serial.println("drive right");
}

void moveMotorR(int pwm,bool dir){
  if( dir ==1){
    digitalWrite(rightDir1,HIGH);
    digitalWrite(rightDir2,LOW);
  }
  else{    
    digitalWrite(rightDir2,HIGH);
    digitalWrite(rightDir1,LOW);
  }
  analogWrite(rightspd,pwm);
  //Serial.println("drive left");
}
