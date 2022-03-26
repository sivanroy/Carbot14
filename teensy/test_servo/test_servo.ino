/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <DynamixelSerial.h>
int ControlPin = 1;
int ID5 = 5;
const int switchPin = 2;         // the number of the switch pin
int switchState = 0; 
String data = "NULL";
int pushed = 0;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             60
// our servo # counter
uint8_t servonum = 0;

void setup() {
  //Serial.begin(9600);
  //Serial.println("16 channel PWM test!");
  Dynamixel.setSerial(&Serial3); // &Serial - Arduino UNO/NANO/MICRO, &Serial1, &Serial2, &Serial3 - Arduino Mega
  Dynamixel.begin(57600,ControlPin);  // Inicialize the servo at 1 Mbps and Pin Control 1
  delay(1000);

  pwm.begin();

  //pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(FREQUENCY);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  //Wire.setClock(400000);
  pwm.setPWM(0, 0, pulseWidth(23));
  Serial.println("Start postion servo 0");

  pwm.setPWM(1, 0, pulseWidth(174));
  Serial.println("Start position servo 1");
  
  pwm.setPWM(2, 0, pulseWidth(180));
  Serial.println("Set servo 2");

  pwm.setPWM(3, 0, pulseWidth(100));
  Serial.println("Set servo 3");

  pinMode(switchPin, INPUT_PULLUP);
  Dynamixel.setEndless(ID5,ON);
  
}

void loop() {
  /*
  delay(4000);
  pwm.setPWM(3, 0, pulseWidth(-10));
  Serial.print("0\n");
  delay(2000);
  pwm.setPWM(3, 0, pulseWidth(100));
  Serial.print("180\n");
  */
  /*
  Dynamixel.moveSpeed(ID5,500,35);
  Serial.print("0\n");
  delay(5000);;
  */
  switchState = digitalRead(switchPin);
    
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.println(data);
  }

  if (switchState == HIGH) {
    Serial.println("Switch is ON");
  }
  if (switchState == LOW) {
    Serial.println("Switch is OFF");
  }
  
  Dynamixel.turn(ID5,RIGTH,0);  

  if (data == "Activate dynamixel" && pushed == 0){
    Dynamixel.turn(ID5,RIGTH,700);
    delay(1550);
    Dynamixel.turn(ID5,LEFT,700);
    delay(1625);
    Dynamixel.turn(ID5,RIGTH,0);
    data = "NULL";
    pushed = 1;
  }
  
  /*
  pwm.setPWM(2, 0, pulseWidth(0));
  delay(1000);
  pwm.setPWM(2, 0, pulseWidth(180));
  delay(1000);
  */
  
  /*
  pwm.setPWM(0, 0, pulseWidth(210));
  Serial.println("End");

  pwm.setPWM(1, 0, pulseWidth(-7));
  Serial.println("End");
  delay(2000);

  pwm.setPWM(0, 0, pulseWidth(23));
  Serial.println("Start");

  pwm.setPWM(1, 0, pulseWidth(174));
  Serial.println("Start");
  delay(2000);
  */
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
