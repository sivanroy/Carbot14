#include <DynamixelSerial.h>
int ControlPin = 1;
int ID = 1;
int ID1 = 11;
int cond = 0;
int Temperature,Voltage,Position; 

void setup(){
Dynamixel.setSerial(&Serial3); // &Serial - Arduino UNO/NANO/MICRO, &Serial1, &Serial2, &Serial3 - Arduino Mega
Dynamixel.begin(57600,ControlPin);  // Inicialize the servo at 1 Mbps and Pin Control 1
delay(1000);

}

void loop(){
  if (cond == 0) {
    cond = cond + 1;
  }
  Dynamixel.setEndless(ID,ON);
  Dynamixel.setEndless(ID1,ON);
  Dynamixel.turn(ID,RIGTH,500);
  Dynamixel.turn(ID1,RIGTH,500);
  delay(500);
  Dynamixel.setEndless(ID,OFF);
  Dynamixel.setEndless(ID1,OFF);
  delay(2000);
  /*
  Dynamixel.moveSpeed(ID,250,50);
  Serial.print("0\n");
  delay(2000);
  Dynamixel.moveSpeed(ID,600,50);
  Serial.print("512\n");
  delay(2000);
  */
  /*
  Position = Dynamixel.readPosition(ID);

  Dynamixel.end();                 // End Servo Comunication
  Serial.begin(9600);
  Serial.print("Position: ");
  Serial.print(Position);
  Serial.end();                     // End the Serial Comunication
  Dynamixel.begin(57600,ControlPin);         // Begin Servo Comunication

  delay(1000);
  */
  /*
  Dynamixel.move(ID,random(200,800));  // Move the Servo radomly from 200 to 800
  Dynamixel.move(ID1,random(200,800));
  delay(1000);
  Dynamixel.moveSpeed(ID,random(200,800),random(200,800));
  Dynamixel.moveSpeed(ID1,random(200,800),random(200,800));
  delay(2000);
  Dynamixel.setEndless(ID,ON);
  Dynamixel.turn(ID,RIGTH,1000);
  Dynamixel.setEndless(ID1,ON);
  Dynamixel.turn(ID1,RIGTH,1000);
  delay(3000);
  Dynamixel.turn(ID,LEFT,1000);
  Dynamixel.turn(ID1,LEFT,1000);
  delay(3000);
  Dynamixel.setEndless(ID,OFF);
  Dynamixel.ledStatus(ID,ON);
  Dynamixel.setEndless(ID1,OFF);
  Dynamixel.ledStatus(ID1,ON);
  
  Dynamixel.moveRW(ID,512);
  Dynamixel.moveRW(ID1,512);
  delay(1000);
  Dynamixel.action();
  Dynamixel.ledStatus(ID,OFF);
  Dynamixel.ledStatus(ID1,OFF);
  */
}
