#include <DynamixelSerial.h>
int ControlPin = 1;
int ID = 4;

void setup(){
Dynamixel.setSerial(&Serial3); 
Dynamixel.begin(57600,ControlPin);
Dynamixel.setEndless(ID,ON);
}

void loop(){
  Dynamixel.moveSpeed(ID,250,300);
  Serial.print("0\n");
  delay(2000);
  Dynamixel.moveSpeed(ID,600,300);
  Serial.print("512\n");
  delay(2000);
  Dynamixel.setEndless(ID,OFF);
  
  Dynamixel.move(ID,random(200,800));  
  delay(1000);
  
  Dynamixel.moveSpeed(ID,random(200,800),random(200,800));
  delay(2000);
  
  Dynamixel.moveRW(ID,512);
  delay(1000);
  Dynamixel.action();
  delay(1000);
}
