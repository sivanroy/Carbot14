
#include <Wire.h> //Servos
#include <Adafruit_PWMServoDriver.h> //Servos
#include <DynamixelSerial.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //Servos

#define MIN_PULSE_WIDTH       650 //Servos
#define MAX_PULSE_WIDTH       2350 //Servos
#define DEFAULT_PULSE_WIDTH   1500 //Servos
#define FREQUENCY             60 //Servos

int ControlPin = 1;
int ID5 = 5;
const int switchPin = 2;
const int pumpPin = 3;
int switchState = 0; 
String data = "NULL";
int pushed = 0;

void setup() {
  Dynamixel.setSerial(&Serial3);
  Dynamixel.begin(57600,ControlPin);
  delay(1000);
  
  pwm.begin();
  Serial.begin(9600);
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(0, 0, pulseWidth(23));
  pwm.setPWM(1, 0, pulseWidth(174));
  pwm.setPWM(2, 0, pulseWidth(180));
  pwm.setPWM(3, 0, pulseWidth(100));
  
  pinMode(switchPin, INPUT_PULLUP);

  Dynamixel.setEndless(ID5,ON);
}

void loop() {
  checkRPI();
  switches();
  pushUnderTheShed();
}

void checkRPI(){
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }
}
void pushUnderTheShed(){
  if (data == "Activate dynamixel" && pushed == 0){
    Dynamixel.turn(ID5,RIGTH,700);
    delay(1550);
    Dynamixel.turn(ID5,LEFT,700);
    delay(1625);
    Dynamixel.turn(ID5,RIGTH,0);
    data = "NULL";
    pushed = 1;
  }
}

bool switches(){
  switchState = digitalRead(switchPin);
  
  if (switchState == HIGH) {
    Serial.println("Switch is ON");
    return true;
  } 
  if (switchState == LOW) {
    Serial.println("Switch is OFF");
    return false;
  }
  return false;
}

/*
void pump() {
  delay(5000);
  digitalWrite(pumpPin, HIGH);
  delay(5000);
  digitalWrite(pumpPin, LOW);
}

void servos(){
  pwm.setPWM(0, 0, pulseWidth(0));
  Serial.println("0");

  pwm.setPWM(1, 0, pulseWidth(0));
  Serial.println("0");
  delay(500);

  pwm.setPWM(0, 0, pulseWidth(180));
  Serial.println("180");

  pwm.setPWM(1, 0, pulseWidth(180));
  Serial.println("180");
  delay(500);
}
*/
int pulseWidth(int angle) // For servos
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
