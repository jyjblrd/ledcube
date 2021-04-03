#include <FastLED.h>
#include <ADXL345.h>
#include <Wire.h>

#define DATA_PIN    7
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    96
CRGB leds[NUM_LEDS];

#define BRIGHTNESS         8
#define FRAMES_PER_SECOND  120

ADXL345 adxl;
enum InterruptMask {
    OVERRUN    = B00000001,
    WATERMARK  = B00000010,
    FREE_FALL  = B00000100,
    INACTIVITY = B00001000,
    ACTIVITY   = B00010000,
    DOUBLE_TAP = B00100000,
    SINGLE_TAP = B01000000,
    DATA_READY = B10000000
};

void setup() {
  Serial.begin(1000000);

  // prevevent resetting on interrupt
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
  
void loop()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically

  // Check battery voltage
  EVERY_N_SECONDS( 1 ) {
    double batVoltage = analogRead(A0)*0.004525386313;
    Serial.printf("Voltage: %.3f\n", batVoltage);  

    if (batVoltage <= 3.3) {
      flashLights(CHSV(0, 255, 255));
      enterDeepSleep();
    }
  }
}

void enterDeepSleep() {
  Serial.println("Entering Deep Sleep");

  FastLED.clear();
  FastLED.show();
  ESP.deepSleep(0);
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void flashLights(CHSV color) {
  // Flash lights
  fill_solid( leds, NUM_LEDS, color);
  FastLED.show();
  delay(100);
  FastLED.clear();
  FastLED.show();
  delay(100);
  fill_solid( leds, NUM_LEDS, color);
  FastLED.show();
  delay(100);
  FastLED.clear();
  FastLED.show();
}