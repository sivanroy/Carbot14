//Libraries
#include <FastLED.h>//https://github.com/FastLED/FastLED

//Constants
#define NUM_STRIPS 1
#define NUM_LEDS 60
#define BRIGHTNESS 10
#define LED_TYPE WS2812B
#define COLOR_ORDER BRG//RGB
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#define FRAMES_PER_SECOND 60
#define COOLING 55
#define SPARKING 120

//Parameters
const int stripPin  = 3;

//Variables
bool gReverseDirection  = false;

//Objects
CRGB leds[NUM_LEDS];

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  //Init led strips
  FastLED.addLeds<LED_TYPE, stripPin, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(  BRIGHTNESS );
}

void loop() {
  ledScenario();
}

void ledScenario(void ) { /* function ledScenario */
  ////LEDS Strip scenario
  for (int i = 0; i < NUM_LEDS; i++) {
    //leds[i] = CRGB::Goldenrod;
    leds[i].setRGB(255, 0, 255);
    leds[i + 3].setRGB(255, 0, 255);
    FastLED.show();
    delay(100);
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(100);
  }
}
