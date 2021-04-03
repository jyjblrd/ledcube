#include <Wire.h>
#include <ADXL345.h>
#include <FastLED.h>    

#define LED_PIN     7
#define NUM_LEDS    96
#define BRIGHTNESS  8
CRGB leds[NUM_LEDS];

ADXL345 adxl;
int rate = 100;

void setup() {
  Serial.begin(1000000);

  // prevevent resetting on interrupt
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  adxl.powerOn();
  adxl.setRate(rate);

  // setup FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // calibrate
  calibrate();
}

void loop(){
}

void calibrate() {
  int dataPoints = 10;
  adxl.setAxisOffset(0, 0, 0);

  // threshold for determining orientation
  int threshold = 50;

  bool hasReachedOrientation[] = {false, false, false, false, false, false}; // x+, x-, y+, y-. z+, z- (facing upwards)
  int minMaxValues[6]; // min and max values for axis. xMin, xMax, yMin, yMax, zMin, zMax
  // int minMaxValues[] = {-257, 253, -243, 271, -256, 235};
  // while has not reached all orientations yet
  while(
    !hasReachedOrientation[0] ||
    !hasReachedOrientation[1] ||
    !hasReachedOrientation[2] ||
    !hasReachedOrientation[3] ||
    !hasReachedOrientation[4] ||
    !hasReachedOrientation[5]
  ) {
    waitForRest();

    // calculate average xyz value
    int xyz[3];
    int xyzSum[] = {0, 0, 0};
    int xyzAvg[3];
    for (size_t i = 0; i < dataPoints; i++) {
      adxl.readAccel(xyz); // readAccel gets xyz for some reason?
      for (size_t i = 0; i < 3; i++) {
        xyzSum[i] += xyz[i];
      }
      delay(1000/rate);
    }
    for (size_t i = 0; i < 3; i++) {
      xyzAvg[i] = xyzSum[i]/dataPoints;
    }

    // x+ orientation
    if (!hasReachedOrientation[0] && xyzAvg[0] > threshold && xyzAvg[1] < threshold && xyzAvg[2] < threshold) {
      Serial.println("x+");
      hasReachedOrientation[0] = true;
      minMaxValues[1] = xyzAvg[0];
      flashLights(CHSV(0, 255, 255));
    }
    // x- orientation
    else if (!hasReachedOrientation[1] && xyzAvg[0] < -threshold && xyzAvg[1] < threshold && xyzAvg[2] < threshold) {
      Serial.println("x-");
      hasReachedOrientation[1] = true;
      minMaxValues[0] = xyzAvg[0];
      flashLights(CHSV(0, 255, 255));
    }
    // y+ orientation
    else if (!hasReachedOrientation[2] && xyzAvg[0] < threshold && xyzAvg[1] > threshold && xyzAvg[2] < threshold) {
      Serial.println("y+");
      hasReachedOrientation[2] = true;
      minMaxValues[3] = xyzAvg[1];
      flashLights(CHSV(0, 255, 255));
    }
    // y- orientation
    else if (!hasReachedOrientation[3] && xyzAvg[0] < threshold && xyzAvg[1] < -threshold && xyzAvg[2] < threshold) {
      Serial.println("y-");
      hasReachedOrientation[3] = true;
      minMaxValues[2] = xyzAvg[1];
      flashLights(CHSV(0, 255, 255));
    }
    // z+ orientation
    else if (!hasReachedOrientation[4] && xyzAvg[0] < threshold && xyzAvg[1] < threshold && xyzAvg[2] > threshold) {
      Serial.println("z+");
      hasReachedOrientation[4] = true;
      minMaxValues[5] = xyzAvg[2];
      flashLights(CHSV(0, 255, 255));
    }
    // z- orientation
    else if (!hasReachedOrientation[5] && xyzAvg[0] < threshold && xyzAvg[1] < threshold && xyzAvg[2] < -threshold) {
      Serial.println("z-");
      hasReachedOrientation[5] = true;
      minMaxValues[4] = xyzAvg[2];
      flashLights(CHSV(0, 255, 255));
    }
  }

  Serial.printf("xMin: %d xMax: %d yMin: %d yMax: %d zMin: %d zMax: %d\n", minMaxValues[0], minMaxValues[1], minMaxValues[2], minMaxValues[3], minMaxValues[4], minMaxValues[5]);

  // calculate gains
  double gains[3];
  for (size_t i = 0; i < 3; i++) {
    gains[i] = 2.0/(-minMaxValues[i*2] + minMaxValues[i*2+1]);
  }

  // calclate offsets
  int offsets[3];
  for (size_t i = 0; i < 3; i++) {
    int posOffset = minMaxValues[i*2+1] - (1.0/gains[i]);
    int negOffset = (1.0/gains[i]) + minMaxValues[i*2];
    offsets[i] = (((posOffset + negOffset)/2.0) * gains[i]) / 15.6e-3;
  }
  
  // write to accelerometer
  adxl.setAxisGains(gains);
  adxl.setAxisOffset(-offsets[0], -offsets[1], -offsets[2]);
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

void waitForRest() {
  double restThreshold = 0.05;
  double delayTime = 100;

  double lastAccels[3];
  double accelDeltas[] = {100, 100, 100};
  adxl.getAcceleration(lastAccels);

  int timesAtRest = 0;

  while (
    timesAtRest < 10
  ) {
    delay(delayTime);

    double accels[3];
    adxl.getAcceleration(accels);
    for (size_t i = 0; i < 3; i++) {
      accelDeltas[i] = fabs(accels[i] - lastAccels[i]);
    }
    memcpy(lastAccels, accels, sizeof(accels));
    
    if (
      accelDeltas[0] < restThreshold &&
      accelDeltas[1] < restThreshold &&
      accelDeltas[2] < restThreshold
    ) {
      timesAtRest++;
    }
    else {
      timesAtRest = 0;
    }
  }
}