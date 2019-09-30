#include <Servo.h>

Servo armServo;
Servo clawServo;

byte inputByte[2] = {0,0};
int command = 0;
bool direction;
int distance = 0;
bool commandReceived = false;

const int leftDir = 7;
const int leftPulse = 4;
const int rightDir = 6;
const int rightPulse = 3;

const int motorEnable = 8;

const int endLed = 2;

const int armJoint = 12;
const int grabJoint = 13;
const int clamp_close = 35; 
const int clamp_open = 10;

const int armUp = 155;
const int armStraignt = 85;
const int armBlock = 60;

const int maxTime = 6000;
const int minTime = 3500;
const int slope = 2500/30;

// unitConversion = 2*pi*(Wheel Radius)*(step degrees/360)
const float unitConversion = 0.141372;

void setup() {
  Serial.begin(9600);

  pinMode(leftDir, OUTPUT);
  pinMode(leftPulse, OUTPUT);
  pinMode(rightDir, OUTPUT);
  pinMode(rightPulse, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  pinMode(endLed, OUTPUT);

  digitalWrite(motorEnable, LOW);
  digitalWrite(endLed, LOW); 

  armServo.attach(armJoint);
  clawServo.attach(grabJoint);
  
  movArm(armUp);
  grab(0);
}

void loop() {

    if(Serial.available() > 0){
      inputByte[0] = Serial.read();
      inputByte[1] = Serial.read();
      commandReceived = true;


      processor(inputByte[0], inputByte[1]);
   }
  
  if(commandReceived){
    if(bitRead(inputByte[0], 4) == 1)
      signalEnd();
        
    switch(command){
         case 0:
          mov(distance, direction);
          // Serial.write(255);
          break;

         case 1:
           rotate(distance, direction);
          //  Serial.write(255);
           break;

         case 2:
           movArm(distance);
          //  Serial.write(255);
           break;

   
         case 3:
           grab(direction);
          //  Serial.write(255);
           break; 
      }
    commandReceived = false;
  }

}

void processor (byte firstByte, byte secondByte){
  byte temp = 0;

  //command Bits
  bitWrite(temp, 0, bitRead(firstByte, 6));
  bitWrite(temp, 1, bitRead(firstByte, 7));
  command = (int)temp;

  //direction bit
  direction = bitRead(firstByte, 5);

  //distance Bits
  distance = 0;
  distance = (int)secondByte;
  int temp2 = 0;
  for(int c = 0; c < 4; c++){
    temp2 = bitRead(firstByte, c);
    distance += temp2*pow(2, c+8) + (temp2*1);
  }
}

void mov(int units, bool directionSetting){
  int pulseCount = (int) units/unitConversion;
  digitalWrite(leftDir, directionSetting);
  digitalWrite(rightDir, directionSetting);

  int time;
  int rampDownStart = maxTime-(slope*pulseCount);

  for(int c = 0; c<pulseCount; c++){

    if(c<pulseCount/2){
      time=maxTime - slope*c;
      if(time<minTime)
        time = minTime;
    }
    else if(c>pulseCount/2){
      time=(slope*c) + rampDownStart;
      if(c<(pulseCount-(2500/slope)))
        time = minTime;
    }
    
    digitalWrite(rightPulse, HIGH);
    digitalWrite(leftPulse, HIGH);
    delayMicroseconds(time);
    digitalWrite(rightPulse, LOW); 
    digitalWrite(leftPulse, LOW);
    delayMicroseconds(time);
  }
  return;
}

void rotate(int deg, bool directionSetting){
  int pulseCount = (int) (2*3.1415*8.6*deg)/(360*unitConversion);

  digitalWrite(leftDir, direction);
  digitalWrite(rightDir, !direction);

  int time;
  int rampDownStart = maxTime-(slope*pulseCount);

  for(int c = 0; c < pulseCount; c++){

    if(c<pulseCount/2){
      time=maxTime - slope*c;
      if(time<minTime)
        time = minTime;
    }
    else if(c>pulseCount/2){
      time=(slope*c) + rampDownStart;
      if(c<(pulseCount-(2500/slope)))
        time = minTime;
    }

    digitalWrite(leftPulse, HIGH);
    digitalWrite(rightPulse, HIGH);
    delayMicroseconds(time);
    digitalWrite(leftPulse, LOW);
    digitalWrite(rightPulse, LOW);
    delayMicroseconds(time);
  }
  return;
}

void movArm(int angle){
  if( angle > 180 || angle < 0){
    return;
  }
  armServo.write(angle);
  return;
}

void grab(bool clamp){
  if(clamp)
    clawServo.write(clamp_close);

  else
    clawServo.write(clamp_open);

return;
}

void signalEnd(){
    while(true){
      digitalWrite(endLed, HIGH);
    }
  return;
}
