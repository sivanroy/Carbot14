
/*
Communication Teensy -> RPI :
1 -> Pallet violet (470 Ohm)
2 -> Pallet jaune (1 kOhm)
3 -> Pallet rouge (4.7 kOhm)
4 -> Back switch ON
5 -> Front switch ON

Communication RPI -> Teensy :
5 -> Push under the shed (Activate dyna ID5 OUT)
6 -> Push under the shed back under the robot (Activate dyna ID5 IN)
R -> Reset all global variables to 0 (undo)
A -> Both front servo OUT : hold pallets to push under the shed
B -> Both front servo IN : release pallets to push under the shed
C -> Measure resistance
D -> Push cube
F -> Take 1st pallet from stack and drop it at the bottom of expoisiton gallery (pos: 100)
G -> Take 2nd pallet from stack and drop it at the bottom of expoisiton gallery (pos: 160)
H -> Take 3rd pallet from stack and drop it at the bottom of expoisiton gallery (pos: 220)
J
I
K -> Lower flip to take pallets from distributor (pos: 255)
L -> Lift the pallets 90 degrees with the flip (pos: 590)
M -> Put the flip in initial position (pos: 755)
N -> Take 1st pallet from stack and drop it at the top of expositon gallery (pos: 100)
O -> Take 1st pallet from stack and drop it at the top of expositon gallery (pos: 160)
P -> Take 1st pallet from stack and drop it at the top of expositon gallery (pos: 220)
Q -> Clamp IN -> release the statuette
S -> Clamp OUT -> clamps the statuette
U -> Push the measure resistance pallet 
*/

#include <Wire.h> //I2C
#include <Adafruit_PWMServoDriver.h> //Servos
#include <DynamixelSerial.h>
#include <Tic.h> //Stepper

#define MIN_PULSE_WIDTH       650 //Servos
#define MAX_PULSE_WIDTH       2350 //Servos
#define DEFAULT_PULSE_WIDTH   1500 //Servos
#define FREQUENCY             60 //Servos

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //Servos
TicI2C tic(0x0E); //Stepper

const int controlPin = 1;
const int ID1 = 1;
const int ID4 = 4;
const int ID5 = 5;

int servoIn0 = -14; //Right front servo is IN at position -12
int servoOut0 = 168; //Right front servo is OUT at position 168
int servoIn1 = 200; //Left front 
int servoOut1 = 13; 
int servoIn2 = 180; //Measure res
int servoOut2 = 15;
int servoIn3 = 130; //Push cube
int servoOut3 = 195;
int servoIn4 = -34; //Clamp
int servoOut4 = 55;
int servoIn5 = 185; //MAX : 200 //Push resistance
int servoOut5 = 35; //MIN : -25

int delta = -245;
int flip3P = 670 + delta ;
int flip2P = 660 + delta;
int flip1P = 645 + delta;
int flipUp = 820 + delta;
int flipDown = 295 + delta;

const int measureResPin = 21;
int rawMeasure = 0;
int Vin = 3.3;
float Vout = 0;
float Rn = 1000;
float R = 0;
float buffer = 0;

const int switchArmPin = 2;
const int switchFrontPin = 3;
const int switchBackPin = 4;
const int pumpPin = 5;
const int valvePin = 7;
int switchArmState = 0; 
int switchFrontState = 0; 
int switchBackState = 0; 
String data = "NULL";

int pushedUnderTheShed = 0; // 1 if it has been pushed
int pushedUnderTheShedBis = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Dynamixel.setSerial(&Serial3);
  digitalWrite(9, HIGH);
  Dynamixel.begin(57600,controlPin);
 
  pinMode(switchArmPin, INPUT_PULLUP);
  pinMode(switchFrontPin, INPUT_PULLUP);
  pinMode(switchBackPin, INPUT_PULLUP);
  
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW);
  
  setArm();
  setServos();
  
  Dynamixel.setEndless(ID1,OFF);
  Dynamixel.setEndless(ID4,OFF);
  Dynamixel.setEndless(ID5,ON);
  
  delay(1000);
}

void loop() {
  checkRPI();
  resetArm();
  resetVariables();
  frontSwitch();
  backSwitch();
  pushUnderTheShed();
  clamp();
  frontServos();
  pushCube();
  pushPallet();
  servoResistance();
  measureResistance();
  moveStepper();
  flip();
  
  delay(10);
}

void clamp(){
   if (data == "Q"){
    pwm.setPWM(4, 0, pulseWidth(servoIn4));
    data = "NULL";
   }
   if (data == "S"){
    pwm.setPWM(4, 0, pulseWidth(servoOut4));
    data = "NULL";
   }
}


void measureResistance(){
  rawMeasure = analogRead(measureResPin);
  if(rawMeasure < 900){
    buffer = rawMeasure * Vin;
    Vout = (buffer)/1024.0;
    R = Rn * Vout/(Vin-Vout);
    if (R > 250 && R < 750){
      pwm.setPWM(5, 0, pulseWidth(servoOut5));
      delay(950);
      pwm.setPWM(5, 0, pulseWidth(servoIn5));
      delay(1500);
      Serial.print("1");
    }
    if (R > 750 && R < 1500){
      Serial.print("2");
    }
    if (R > 1500 && R < 6000){
      Serial.print("3");
    }
  }
}

void flip(){
  if (data == "K"){
    Dynamixel.moveSpeed(ID4,flipDown,160);
    data = "NULL";
  }
  if (data == "L"){
    Dynamixel.moveSpeed(ID4,flip3P,512);
    data = "NULL";
  }
  if (data == "M"){
    Dynamixel.moveSpeed(ID4,flipUp,512);
    data = "NULL";
  }
} 

void moveStepper(){
  setArm();
  
  if (data == "F"){
    Dynamixel.moveSpeed(ID4,flip3P,1023);
    delay(2000);
  
    Wire.beginTransmission(0x0E);
    goToPosition(105);
    digitalWrite(pumpPin, HIGH);
    delay(750);
    goToPosition(0);
    delay(500);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    goToPosition(350);
    delay(1000);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    goToPosition(0);
    delay(1000);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission();  
    data = "NULL";
  }
  
  
  if (data == "G"){
    Dynamixel.moveSpeed(ID4,flip2P,1023);
    delay(2000);
  
    Wire.beginTransmission(0x0E);
    goToPosition(165);
    digitalWrite(pumpPin, HIGH);
    delay(1000);
    goToPosition(0);
    delay(750);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    goToPosition(350);
    delay(1000);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    goToPosition(0);
    delay(1000);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission();
    data = "NULL";
  }

  if (data == "H"){
    Dynamixel.moveSpeed(ID4,flip1P,1023);
    delay(2000);
  
    Wire.beginTransmission(0x0E);
    goToPosition(230);
    digitalWrite(pumpPin, HIGH);
    delay(1250);
    goToPosition(0);
    delay(1000);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    goToPosition(350);
    delay(1000);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    goToPosition(0);
    delay(1000);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission(); 
    data = "NULL";
  }
  
  if (data == "N"){
    Wire.beginTransmission(0x0E);
    goToPosition(105);
    digitalWrite(pumpPin, HIGH);
    delay(750);
    goToPosition(0);
    delay(500);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission();  
    data = "NULL";
  }
  
  
  if (data == "O"){
    Dynamixel.moveSpeed(ID4,flip2P,1023);
    delay(2000);
    Wire.beginTransmission(0x0E);
    goToPosition(165);
    digitalWrite(pumpPin, HIGH);
    delay(1000);
    goToPosition(0);
    delay(750);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission();
    data = "NULL";
  }

  if (data == "P"){
    Dynamixel.moveSpeed(ID4,flip1P,1023);
    delay(2000);
  
    Wire.beginTransmission(0x0E);
    goToPosition(230);
    digitalWrite(pumpPin, HIGH);
    delay(1250);
    goToPosition(0);
    delay(1000);
    Dynamixel.moveSpeed(ID1,600,1023);
    delay(500);
    digitalWrite(valvePin, HIGH);
    delay(50);
    digitalWrite(pumpPin, LOW);
    digitalWrite(valvePin, LOW);
    Dynamixel.moveSpeed(ID1,820,1023);
    Wire.endTransmission(); 
    data = "NULL";
  }
}

void goToPosition(int pos){
  tic.setTargetPosition(pos);
  while(tic.getTargetPosition()!= pos){
      tic.setTargetPosition(pos);
      if (pos == 0 && armSwitch()){
        Serial.println("switch");
        break;
      }
   } 
}

void checkRPI(){
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
  }
}

void resetVariables(){
  if (data == "R"){
    pushedUnderTheShed = 0;
    pushedUnderTheShedBis = 0;
    data = "NULL";
  }
}


void pushUnderTheShed(){
  if (data == "5" && pushedUnderTheShed == 0){
    Dynamixel.turn(ID5,RIGTH,1023);
    delay(1100);
    Dynamixel.turn(ID5,LEFT,0);
    data = "NULL";
    pushedUnderTheShed = 1;
  }
  if (data == "6" && pushedUnderTheShedBis == 0){
    Dynamixel.turn(ID5,LEFT,1023);
    pwm.setPWM(4, 0, pulseWidth(servoOut4));
    delay(1180);
    Dynamixel.turn(ID5,RIGTH,0);
    data = "NULL";
    pushedUnderTheShedBis = 1;
  }
}

bool frontSwitch(){
  switchFrontState = digitalRead(switchFrontPin);
  
  if (switchFrontState == HIGH) {
    Serial.print("5");
    return true;
  } 
  if (switchFrontState == LOW) {
    Serial.print("0");
    return false;
  }
  return false;
}

bool backSwitch(){
  switchBackState = digitalRead(switchBackPin);
  
  if (switchBackState == HIGH) {
    Serial.print("4");
    return true;
  } 
  if (switchBackState == LOW) {
    return false;
  }
  return false;
}

bool armSwitch(){
  switchArmState = digitalRead(switchArmPin);
  if (switchArmState == HIGH) {
    //Serial.print("O");
    return true;
  } 
  if (switchArmState == LOW) {
    //Serial.print("E");
    return false;
  }
  return false;
}

void setArm(){
  if (armSwitch()){
    Wire.beginTransmission(0x0E);
    tic.haltAndSetPosition(0);
    tic.exitSafeStart();
    Wire.endTransmission();
  }
  else{
    Serial.print("OFF\n");
    Wire.beginTransmission(0x0E);
    tic.haltAndSetPosition(0);
    tic.exitSafeStart();
    while (armSwitch() != true){
      tic.setTargetVelocity(-5000000);
    }
    tic.haltAndSetPosition(0);
    tic.setTargetPosition(-5);
    waitForPosition(-5);
    tic.haltAndSetPosition(0);
    Wire.endTransmission();
  }
  Dynamixel.moveSpeed(ID1,820,512);
}

void resetArm(){
  if (armSwitch()){
  }
  else{
    Wire.beginTransmission(0x0E);
    tic.haltAndSetPosition(0);
    while (armSwitch() != true){
      tic.setTargetVelocity(-5000000);
    }
    tic.haltAndSetPosition(0);
    tic.setTargetPosition(-5);
    waitForPosition(-5);
    tic.haltAndSetPosition(0);
    Wire.endTransmission();
  }
  Dynamixel.moveSpeed(ID1,820,512);
}

void setServos(){
  Wire.beginTransmission(0x40);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(0, 0, pulseWidth(servoIn0));
  pwm.setPWM(1, 0, pulseWidth(servoIn1));
  pwm.setPWM(2, 0, pulseWidth(servoIn2));
  pwm.setPWM(3, 0, pulseWidth(servoIn3));
  pwm.setPWM(4, 0, pulseWidth(servoIn4));
  pwm.setPWM(5, 0, pulseWidth(servoIn5));
  Wire.endTransmission();
}

void frontServos(){
  if (data == "A"){
    Wire.beginTransmission(0x40);
    pwm.setPWM(0, 0, pulseWidth(servoOut0));
    pwm.setPWM(1, 0, pulseWidth(servoOut1));
    Wire.endTransmission();
    data = "NULL";
  }

  if (data == "B"){
     Wire.beginTransmission(0x40);
     pwm.setPWM(0, 0, pulseWidth(servoIn0));
     pwm.setPWM(1, 0, pulseWidth(servoIn1));
     Wire.endTransmission();
     data = "NULL";
  }
}

void servoResistance(){
  if (data == "C"){
      Wire.beginTransmission(0x40);
      pwm.setPWM(2, 0, pulseWidth(servoOut2));
      delay(2000);
      pwm.setPWM(2, 0, pulseWidth(servoIn2));
      Wire.endTransmission();
      data = "NULL";
  }
}

void pushPallet(){
  if (data == "U"){
    pwm.setPWM(5, 0, pulseWidth(servoOut5));
    delay(950);
    pwm.setPWM(5, 0, pulseWidth(servoIn5));
    delay(1500);
  }
}
  
void pushCube(){
   if (data == "D"){
      pwm.setPWM(3, 0, pulseWidth(servoOut3));
      delay(500);
      pwm.setPWM(3, 0, pulseWidth(servoIn3));
      data = "NULL";
  }
}


int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void resetCommandTimeout(){
  tic.resetCommandTimeout();
}

void delayWhileResettingCommandTimeout(uint32_t ms){
  uint32_t start = millis();
  do
  {
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}

void waitForPosition(int32_t targetPosition){
  do
  {
    resetCommandTimeout();
  } while (tic.getCurrentPosition() != targetPosition);
}
