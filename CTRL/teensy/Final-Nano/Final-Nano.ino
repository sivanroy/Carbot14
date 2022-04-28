/*
* Communication RPI -> Arduino:
* 1 -> +1 : + 1 point for each sample removed from a distributor on the team side (including the shared distributor and the work shed);
*           + 1 point for each sample inside the camp;
*           + 1 additional point for each revealed and sorted sample inside the camp;
* 2 -> +2 : + 2 points for installing the statuette on the pedestal during preparation time;
*           + 2 points for installing a display cabinet during preparation time;
* 3 -> +3 : + 3 points for each sample inside the gallery;
*           + 3 additional points for each revealed and sorted sample inside the gallery;
* 5 -> +5 : + 5 points for each revealed excavation square at the team’s colour;
*           + 5 points additional if a least a excavation square at the team’s colour is revealed, and the red square at the team’s side is not revealed;
*           + 5 points for each sample inside the work shed;
*           + 5 points if the statuette is missing from the pedestal at the end of the game;
*           + 5 additional points if the display cabinet is activated;
* 6 -> +6
* A -> +10 : + 10 points if the replica is on the pedestal at the end of the game;
* F -> +15 : + 15 points if the statuette is inside the display cabinet at the end of the game;
* K -> +20 : + 20 points if all the team robots are inside either the camp either the excavation site;
* R -> Reset points to 4
* C -> Display C14
*/

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <FastLED.h>
#define NUM_LEDS 60
#define LED_PIN 2

CRGB leds[NUM_LEDS];

const int BUFFER_SIZE = 1;
char buf[BUFFER_SIZE];

int pinA = 12;
int pinB = 8;
int pinC = 5;
int pinD = 6;
int pinE = 7;
int pinF = 11;
int pinG = 4;
int D1 = 2;
int D2 = 10;
int D3 = 9;
int D4 = 3;

String strPoints = "0000";
int points;
int currDigit = D1;
int len = 0;

int started = 0;
String data = "0";

void setup() {
 Serial.begin(57600);
 FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
 FastLED.setBrightness(130);
 //FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);    
 
 points = 4;
 
 pinMode(pinA, OUTPUT);     
 pinMode(pinB, OUTPUT);     
 pinMode(pinC, OUTPUT);     
 pinMode(pinD, OUTPUT);     
 pinMode(pinE, OUTPUT);     
 pinMode(pinF, OUTPUT);     
 pinMode(pinG, OUTPUT);   
 pinMode(D1, OUTPUT);  
 pinMode(D2, OUTPUT);  
 pinMode(D3, OUTPUT);  
 pinMode(D4, OUTPUT);  
}


void loop() {
 checkRPI();
 count();
 digitDisplay();
 ledStrip();
}

void ledStrip(){
  //
}

void scoreBlink(){
  fill_solid(leds, NUM_LEDS, CRGB::Red); 
  FastLED.show();
  digitDisplay();
  delay(10);
  fill_solid(leds, NUM_LEDS, CRGB::Black); 
  FastLED.show();
  digitDisplay();
  delay(10);
}

void sawTooth(){
  uint8_t sinBeat   = beatsin8(30, 0, NUM_LEDS - 1, 0, 0);
  uint8_t sinBeat2  = beatsin8(30, 0, NUM_LEDS - 1, 0, 85);
  uint8_t sinBeat3  = beatsin8(30, 0, NUM_LEDS - 1, 0, 170);

  // If you notice that your pattern is missing out certain LEDs, you
  // will need to use the higher resolution beatsin16 instead. In this
  // case remove the 3 lines above and replace them with the following:
  // uint16_t sinBeat   = beatsin16(30, 0, NUM_LEDS - 1, 0, 0);
  // uint16_t sinBeat2  = beatsin16(30, 0, NUM_LEDS - 1, 0, 21845);
  // uint16_t sinBeat3  = beatsin16(30, 0, NUM_LEDS - 1, 0, 43690);

  leds[sinBeat]   = CRGB::Blue;
  leds[sinBeat2]  = CRGB::Red;
  leds[sinBeat3]  = CRGB::White;
  
  fadeToBlackBy(leds, NUM_LEDS, 10);

  EVERY_N_MILLISECONDS(10){
    Serial.print(sinBeat);
    Serial.print(",");
    Serial.print(sinBeat2);
    Serial.print(",");
    Serial.println(sinBeat3);
  }

  FastLED.show();
}

void phaseBeat(){
  uint8_t sinBeat   = beatsin8(30, 0, NUM_LEDS - 1, 0, 0);
  uint8_t sinBeat2  = beatsin8(30, 0, NUM_LEDS - 1, 0, 85);
  uint8_t sinBeat3  = beatsin8(30, 0, NUM_LEDS - 1, 0, 170);

  // If you notice that your pattern is missing out certain LEDs, you
  // will need to use the higher resolution beatsin16 instead. In this
  // case remove the 3 lines above and replace them with the following:
  //uint16_t sinBeat   = beatsin16(30, 0, NUM_LEDS - 1, 0, 0);
  //uint16_t sinBeat2  = beatsin16(30, 0, NUM_LEDS - 1, 0, 21845);
  //uint16_t sinBeat3  = beatsin16(30, 0, NUM_LEDS - 1, 0, 43690);

  leds[sinBeat]   = CRGB::Blue;
  leds[sinBeat2]  = CRGB::Red;
  leds[sinBeat3]  = CRGB::White;
  
  fadeToBlackBy(leds, NUM_LEDS, 10);

  EVERY_N_MILLISECONDS(10){
    Serial.print(sinBeat);
    Serial.print(",");
    Serial.print(sinBeat2);
    Serial.print(",");
    Serial.println(sinBeat3);
  }

  FastLED.show();
}

void rainbowBeat(){
  uint16_t beatA = beatsin16(30, 0, 255);
  uint16_t beatB = beatsin16(20, 0, 255);
  fill_rainbow(leds, NUM_LEDS, (beatA+beatB)/2, 8);
  
  FastLED.show();
}

void checkRPI(){
  if (Serial.available() > 0) {
    Serial.readBytes(buf, BUFFER_SIZE);
    data = buf[0];
  }
}

void count() {
  if (data == "1"){
    points = points + 1;
    started = 1;
    data = "0";
    fill_solid(leds, NUM_LEDS, CRGB::Red); 
    FastLED.show();
    digitDisplay();
    delay(10);
    fill_solid(leds, NUM_LEDS, CRGB::Black); 
    FastLED.show();
    digitDisplay();
    delay(10);
  }  
  
  if (data == "2"){
    for (int i=0; i<2; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "3"){
    for (int i=0; i<3; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "5"){
    for (int i=0; i<5; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "6"){
    for (int i=0; i<6; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "A"){
    for (int i=0; i<10; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "F"){
    for (int i=0; i<15; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "K"){
    for (int i=0; i<20; i++){
      points = points + 1;
      started = 1;
      data = "0";
      scoreBlink();
    }
  }
  if (data == "R"){
    points = 4;
    started = 1;
    data = "0";
  }
}

void digitDisplay(){
  strPoints = String(points);
  len = strPoints.length();
  if (len == 1){
    digitalWrite(D1, HIGH);
    digitalWrite(D2, HIGH);
    digitalWrite(D3, HIGH);
  }
  if (len == 2){
    digitalWrite(D1, HIGH);
    digitalWrite(D2, HIGH);
  }
  if (len == 3){
    digitalWrite(D1, HIGH);
  }
  
  for (int i=0; i<len; i++){
    if (len == 1){
      currDigit = D4;
    }
    if (len == 2 && i == 0){
      currDigit = D3;
    }

    if (len == 2 && i == 1){
      currDigit = D4;
    }

    if (len == 3 && i == 0){
      currDigit = D2;
    }

    if (len == 3 && i == 1){
      currDigit = D3;
    }

    if (len == 3 && i == 2){
      currDigit = D4;
    }

    if (len == 4 && i == 0){
      currDigit = D1;
    }

    if (len == 4 && i == 1){
      currDigit = D2;
    }

    if (len == 4 && i == 2){
      currDigit = D3;
    }

    if (len == 4 && i == 3){
      currDigit = D4;
    } 

    if (started == 0 || data == "C"){
      digitalWrite(D1, HIGH);
      digitalWrite(D2, HIGH);
      digitalWrite(D3, HIGH);
      digitalWrite(D4, HIGH);
      
      digitalWrite(D2, LOW);
      
      digitalWrite(pinA, HIGH);   
      digitalWrite(pinB, LOW);   
      digitalWrite(pinC, LOW);   
      digitalWrite(pinD, HIGH);   
      digitalWrite(pinE, HIGH);   
      digitalWrite(pinF, HIGH);   
      digitalWrite(pinG, LOW); 

      delay(5);

      digitalWrite(D1, HIGH);
      digitalWrite(D2, HIGH);
      digitalWrite(D3, HIGH);
      digitalWrite(D4, HIGH);
      
      digitalWrite(D3, LOW);

      digitalWrite(pinA, LOW);   
      digitalWrite(pinB, HIGH);   
      digitalWrite(pinC, HIGH);   
      digitalWrite(pinD, LOW);   
      digitalWrite(pinE, LOW);   
      digitalWrite(pinF, LOW);   
      digitalWrite(pinG, LOW);

      delay(5);

      digitalWrite(D1, HIGH);
      digitalWrite(D2, HIGH);
      digitalWrite(D3, HIGH);
      digitalWrite(D4, HIGH);

      digitalWrite(D4, LOW);

      digitalWrite(pinA, LOW);   
      digitalWrite(pinB, HIGH);   
      digitalWrite(pinC, HIGH);   
      digitalWrite(pinD, LOW);   
      digitalWrite(pinE, LOW);   
      digitalWrite(pinF, HIGH);   
      digitalWrite(pinG, HIGH);

      delay(5);

      started = 0;
    }
    if (started == 1){
      
      if (strPoints.charAt(i) == '0'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
        
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, HIGH);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, LOW); 
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '1'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, LOW);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, LOW);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, LOW);   
        digitalWrite(pinG, LOW);
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '2'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
        
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, LOW);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, HIGH);   
        digitalWrite(pinF, LOW);   
        digitalWrite(pinG, HIGH); 
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '3'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, LOW);   
        digitalWrite(pinG, HIGH);
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '4'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
  
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, LOW);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, LOW);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, HIGH);
  
        delay(5);
      }
      
      if (strPoints.charAt(i) == '5'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, LOW);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, HIGH);
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '6'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, LOW);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, HIGH);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, HIGH); 
  
        delay(5);
      }
      
      if (strPoints.charAt(i) == '7'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, LOW);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, LOW);   
        digitalWrite(pinG, LOW);
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '8'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, HIGH);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, HIGH);
  
        delay(5);
      }
  
      if (strPoints.charAt(i) == '9'){
        digitalWrite(D1, HIGH);
        digitalWrite(D2, HIGH);
        digitalWrite(D3, HIGH);
        digitalWrite(D4, HIGH);
        
        digitalWrite(currDigit, LOW);
  
        digitalWrite(pinA, HIGH);   
        digitalWrite(pinB, HIGH);   
        digitalWrite(pinC, HIGH);   
        digitalWrite(pinD, HIGH);   
        digitalWrite(pinE, LOW);   
        digitalWrite(pinF, HIGH);   
        digitalWrite(pinG, HIGH);
  
        delay(5);
      }
    }
  }
}
