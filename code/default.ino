#include <BasicLinearAlgebra.h>
using namespace BLA;

#include <Wire.h>
#include <ADXL345.h>
#include <FastLED.h>    
#include <SimpleTimer.h>

SimpleTimer checkInterruptsTimer(100);
SimpleTimer checkBatteryTimer(1000);

#define LED_PIN     D7
#define NUM_LEDS    96
#define BRIGHTNESS  32
#define MAX_VOLTS   3.3
#define MAX_MAMPS   1000
CRGB leds[NUM_LEDS];

ADXL345 adxl;
double rate = 1600;

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

double points[6][4][4][3] = {
  {
    {{-14.25, -14.25, -20.5}, {-4.75, -14.25, -20.5}, {4.75, -14.25, -20.5}, {14.25, -14.25, -20.5}},
    {{-14.25, -4.75,  -20.5}, {-4.75, -4.75,  -20.5}, {4.75, -4.75,  -20.5}, {14.25, -4.75,  -20.5}},
    {{-14.25,  4.75,  -20.5}, {-4.75,  4.75,  -20.5}, {4.75,  4.75,  -20.5}, {14.25,  4.75,  -20.5}},
    {{-14.25,  14.25, -20.5}, {-4.75,  14.25, -20.5}, {4.75,  14.25, -20.5}, {14.25,  14.25, -20.5}}
  },
  {
    {{20.5, -14.25, -14.25}, {20.5, -14.25, -4.75}, {20.5, -14.25, 4.75}, {20.5, -14.25, 14.25}},
    {{20.5, -4.75,  -14.25}, {20.5, -4.75,  -4.75}, {20.5, -4.75,  4.75}, {20.5, -4.75,  14.25}},
    {{20.5,  4.75,  -14.25}, {20.5,  4.75,  -4.75}, {20.5,  4.75,  4.75}, {20.5,  4.75,  14.25}},
    {{20.5,  14.25, -14.25}, {20.5,  14.25, -4.75}, {20.5,  14.25, 4.75}, {20.5,  14.25, 14.25}}
  },
  {
    {{14.25, 20.5,  14.25}, {4.75, 20.5,  14.25}, {-4.75, 20.5,  14.25}, {-14.25, 20.5,  14.25}}, 
    {{14.25, 20.5,   4.75}, {4.75, 20.5,  4.75},  {-4.75, 20.5,  4.75},  {-14.25, 20.5,  4.75}},
    {{14.25, 20.5,  -4.75}, {4.75, 20.5, -4.75},  {-4.75, 20.5, -4.75},  {-14.25, 20.5, -4.75}},
    {{14.25, 20.5, -14.25}, {4.75, 20.5, -14.25}, {-4.75, 20.5, -14.25}, {-14.25, 20.5, -14.25}}
  },
  {
    {{-20.5,  14.25, -14.25}, {-20.5,  14.25, -4.75}, {-20.5,  14.25, 4.75}, {-20.5,  14.25, 14.25}},
    {{-20.5,  4.75,  -14.25}, {-20.5,  4.75,  -4.75}, {-20.5,  4.75,  4.75}, {-20.5,  4.75,  14.25}},
    {{-20.5, -4.75,  -14.25}, {-20.5, -4.75,  -4.75}, {-20.5, -4.75,  4.75}, {-20.5, -4.75,  14.25}},
    {{-20.5, -14.25, -14.25}, {-20.5, -14.25, -4.75}, {-20.5, -14.25, 4.75}, {-20.5, -14.25, 14.25}}
  },
  {
    {{-14.25, -20.5,  14.25}, {-4.75, -20.5,  14.25}, {4.75, -20.5,  14.25}, {14.25, -20.5,  14.25}},
    {{-14.25, -20.5,  4.75},  {-4.75, -20.5,  4.75},  {4.75, -20.5,  4.75},  {14.25, -20.5,  4.75}},
    {{-14.25, -20.5, -4.75},  {-4.75, -20.5, -4.75},  {4.75, -20.5, -4.75},  {14.25, -20.5, -4.75}},
    {{-14.25, -20.5, -14.25}, {-4.75, -20.5, -14.25}, {4.75, -20.5, -14.25}, {14.25, -20.5, -14.25}}
  },
  {
    {{-14.25,  14.25, 20.5}, {-4.75,  14.25, 20.5}, {4.75,  14.25, 20.5}, {14.25,  14.25, 20.5}},
    {{-14.25,  4.75,  20.5}, {-4.75,  4.75,  20.5}, {4.75,  4.75,  20.5}, {12.75,  5.5,  20.5}},
    {{-14.25, -4.75,  20.5}, {-4.75, -4.75,  20.5}, {4.75, -4.75,  20.5}, {12.25, -5.5,  20.5}},
    {{-14.25, -14.25, 20.5}, {-4.75, -14.25, 20.5}, {4.75, -14.25, 20.5}, {14.25, -14.25, 20.5}}
  }
};
double rotatedPoints[6][4][4][3];
double flatRotatedPoints[6*4*4][3];

// array 
int numbersFont[10][4][4] = {
  {
    {0, 1, 0, 0},
    {1, 0, 1, 0},
    {1, 0, 1, 0},
    {0, 1, 0, 0}
  }, {
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 0, 0},
    {1, 1, 1, 0}
  }, {
    {1, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 0, 0},
    {1, 1, 1, 0}
  }, {
    {1, 1, 1, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {1, 1, 1, 0}
  }, {
    {1, 0, 0, 0},
    {1, 0, 1, 0},
    {1, 1, 1, 0},
    {0, 0, 1, 0}
  }, {
    {1, 1, 0, 0},
    {1, 1, 1, 0},
    {0, 0, 1, 0},
    {1, 1, 1, 0}
  }, {
    {1, 1, 1, 0},
    {1, 1, 0, 0},
    {1, 0, 1, 0},
    {1, 1, 1, 0}
  }, {
    {1, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 0, 0},
    {1, 0, 0, 0}
  }, {
    {1, 1, 1, 0},
    {1, 1, 1, 0},
    {1, 0, 1, 0},
    {1, 1, 1, 0}
  }, {
    {1, 1, 1, 0},
    {1, 0, 1, 0},
    {1, 1, 1, 0},
    {0, 0, 1, 0}
  },
};

int rendererIndex = 0; // which rendering function is being used
int numOfRenderers = 3;

// print bytes
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

void setup() {
  delay(1000);

  Serial.begin(1000000);

  // prevevent resetting on interrupt
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  adxl.powerOn();
  adxl.setRate(rate);

  // inactivity interrupt
  adxl.setInactivityThreshold(5); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(0);
  adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, ADXL345_INT2_PIN);
  adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 1);

  // double tap interrupt
  adxl.setTapDetectionOnX(1);
  adxl.setTapDetectionOnY(1);
  adxl.setTapDetectionOnZ(1);
  adxl.setTapThreshold(100);
  adxl.setTapDuration(15);
  adxl.setDoubleTapLatency(80);
  adxl.setDoubleTapWindow(200);
  adxl.setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT, ADXL345_INT1_PIN);
  adxl.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterruptLevelBit(0);

  // single tap interrupt
  adxl.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT1_PIN);
  adxl.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, 1);

  // freefall interrupt
  adxl.setFreeFallThreshold(9); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(20); //(20 - 70) recommended - 5ms per increment
  adxl.setInterruptMapping(ADXL345_INT_FREE_FALL_BIT, ADXL345_INT1_PIN);
  adxl.setInterrupt(ADXL345_INT_FREE_FALL_BIT, 1);

  byte interruptSource = adxl.getInterruptSource();

  // setup FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(MAX_VOLTS, MAX_MAMPS);

  // calibrate
  calibrate();
}

void loop(){
  static unsigned long lastLoopTime = 0; 
  Serial.printf("%d %d \n", micros()-lastLoopTime, 1000000/(micros()-lastLoopTime));
  lastLoopTime = micros();

  // get accels
  double accels[3];
  adxl.getAcceleration(accels);

  // calulate angles
  /*
  double angles[3]; // x, y, z
  getAngles(accels, angles);
  */

  // calculate normal vector pointing upwards
  double normalVector[] = {0, 0, 0};
  normalVector[0] = clip(accels[1], -1.0, 1.0);
  normalVector[1] = clip(accels[0], -1.0, 1.0);
  normalVector[2] = clip(-accels[2], -1.0, 1.0);
  double normalVectorLength = length(normalVector);
  normalVector[0] = normalVector[0]/normalVectorLength;
  normalVector[1] = normalVector[1]/normalVectorLength;
  normalVector[2] = normalVector[2]/normalVectorLength;


  /*
  Serial.printf(
    "X: %+.2f, Y: %+.2f, Z: %+.2f, xAngle: %+.1f, yAngle: %+.1f, zAngle: %+.1f, Interrupt: " BYTE_TO_BINARY_PATTERN ", Voltage: %.2f\n",
    normalVector[0], normalVector[1], normalVector[2], angles[0], angles[1], angles[2], BYTE_TO_BINARY(interruptSource), batVoltage
  );
  */

  // rotate points
  double rotationAxis[3] = {normalVector[1], -normalVector[0], 0}; // cross product of normal vector and <0,0,1> is rotation axis
  double angle = acos(normalVector[2]/(length(normalVector))); // angle rotated around the axis is using dot product
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 4; j++) {
      for (size_t k = 0; k < 4; k++) {
        double rotatedPoint[3];
        rotatePoint(points[i][j][k], angle, rotationAxis, rotatedPoints[i][j][k]);
      }
    }
  }

  // flatten array of points
  int index = 0;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 4; j++) {
      for (size_t k = 0; k < 4; k++) {
        flatRotatedPoints[index][0] = rotatedPoints[i][j][k][0];
        flatRotatedPoints[index][1] = rotatedPoints[i][j][k][1];
        flatRotatedPoints[index][2] = rotatedPoints[i][j][k][2];
        index++;
      }
    }
  }
  
  // sort rotated points by height
  int indexs[6*4*4];
  for (size_t i = 0; i < 6*4*4; i++) {
    indexs[i] = i;
  }
  qsort(indexs, 6*4*4, sizeof(indexs[0]), sortByHeight);

  // renderes
  switch (rendererIndex) {
  case 0:
    rainbow(indexs);
    break;
  case 1:
    movingPlane(indexs);
    break;
  case 2:
    liquid(indexs);
    break;
  }

  if (checkBatteryTimer.isReady()) {
    checkBattery();
  }

  if (checkInterruptsTimer.isReady()) {
    checkInterrupts();
  }
}

void checkBattery() {
  // get battery voltage
  double batVoltage = analogRead(A0)*0.004525386313;

  // check for low battery
  if (batVoltage <= 3.2) {
    flashLights(CHSV(0, 255, 255));
    enterDeepSleep();
  }
}

void checkInterrupts() {
  // get interrup getInterruptSource
  byte interruptSource = adxl.getInterruptSource();

  // if double tap, show battery level
  if (interruptSource & DOUBLE_TAP) {
    FastLED.clear();
    FastLED.show();
    delay(30);

    // get battery voltage
    double batVoltage = analogRead(A0)*0.004525386313;  
    
    int firstNumber = batVoltage;
    int secondNumber = 10*(batVoltage-firstNumber);
    int thirdNumber = 100*(batVoltage-firstNumber-secondNumber*0.1);
    int splitNumbers[3] = {firstNumber, secondNumber, thirdNumber};

    // draw numbers
    for (size_t numberIndex = 0; numberIndex < 3; numberIndex++) {
      FastLED.clear();
      for (size_t sideIndex = 0; sideIndex < 6; sideIndex++) {
        for (size_t i = 0; i < 4; i++) {
          for (size_t j = 0; j < 4; j++) {
            leds[16*sideIndex + 4*i + j] = numbersFont[splitNumbers[numberIndex]][i][j] ? CRGB::Green : CRGB::Black;
          }
          Serial.println();
        }
      }
      FastLED.show();
      delay(750);
    }
  }

  // check for freefall
  if (interruptSource & SINGLE_TAP) {
    if (rendererIndex < numOfRenderers-1) {
      rendererIndex++;
    }
    else {
      rendererIndex = 0;
    }
  }

  // check for inactivity
  if (interruptSource & INACTIVITY) {
    flashLights(CHSV(0, 255, 255));
    enterDeepSleep();
  }
}

void rainbow(int* indexs) {
  static double hueOffset = 0; // hue offset
  double hueOffsetDelta = 2;

  double highestPoint = flatRotatedPoints[indexs[0]][2];
  double lowestPoint = flatRotatedPoints[indexs[6*4*4-1]][2];
  double totalHeight = lowestPoint-highestPoint;
  double deltaHue = 255.0/totalHeight; // delta hue

  for (size_t i = 0; i < 6*4*4; i++) {
    double hue = (highestPoint+flatRotatedPoints[i][2])*deltaHue + hueOffset;
    leds[i] = CHSV(hue, 255, 255);
  }
  FastLED.show();

  // change hue offset
  if (hueOffset < 255) {
    hueOffset += hueOffsetDelta;
  }
  else {
    hueOffset = 0;
  }
}

void movingPlane(int* indexs) {
  static int planeDirections[] = {4, 5}; // direction that the plane is moving in. 0: x+, 1: x-, 2: y+, 3: y-, 4: z+, 5: z-
  static int planeDirectionIndex = 0;
  static double planePosition = -40; // from -40 to 40
  double planePositionDelta = 1;
  static double hueOffset = 0; // hue offset
  double hueOffsetDelta = 3;

  double highestPoint = flatRotatedPoints[indexs[0]][2];
  double lowestPoint = flatRotatedPoints[indexs[6*4*4-1]][2];
  double totalHeight = lowestPoint-highestPoint;

  double normalVector[3] = {0, 0, 0}; // normal vector of plane
  double pointOnPlane[3] = {0, 0, 0};
  switch (planeDirections[planeDirectionIndex]) {
  case 0:
    normalVector[0] = 1;
    pointOnPlane[0] = planePosition;
    break;
  case 1:
    normalVector[0] = -1;
    pointOnPlane[0] = -planePosition;
    break;
  case 2:
    normalVector[1] = 1;
    pointOnPlane[1] = planePosition;
    break;
  case 3:
    normalVector[1] = -1;
    pointOnPlane[1] = -planePosition;
    break;
  case 4:
    normalVector[2] = 1;
    pointOnPlane[2] = planePosition;
    break;
  case 5:
    normalVector[2] = -1;
    pointOnPlane[2] = -planePosition;
    break;
  default:
    break;
  }
  
  for (size_t i = 0; i < 6*4*4; i++) {
    // if point is above plane
    if (checkIsAbovePlane(normalVector, pointOnPlane, flatRotatedPoints[i])) {
      leds[i] = CRGB::Black;
    }
    else {
      double distanceFromPlane = abs(
        normalVector[0]*flatRotatedPoints[i][0] + 
        normalVector[1]*flatRotatedPoints[i][1] + 
        normalVector[2]*flatRotatedPoints[i][2] -
        planePosition
      );
      double hue = max((40-distanceFromPlane)*2, 0.0) + hueOffset;
      if (hue > 255) hue -= 255;
      double vibrance = max((40-distanceFromPlane)*6, 0.0);

      leds[i] = CHSV(hue, 255, vibrance);
    }
  }
  FastLED.show();

  // increment plane position, hue offset and plane direction
  if (planePosition < 40) {
    planePosition += planePositionDelta;
  }
  else {
    planePosition = -30;
    if (planeDirectionIndex < sizeof(planeDirections)/sizeof(*planeDirections)-1) {
      planeDirectionIndex++;
    }
    else {
      planeDirectionIndex = 0;
    }
  }

  // change hue offset
  if (hueOffset < 255) {
    hueOffset += hueOffsetDelta;
  }
  else {
    hueOffset -= 255;
  }
}

void liquid(int* indexs) {
  double percentFilled = 0.75; // what percent of the cube is filled
  double antialiasingHeight = 10; // how many mm below liquid level to start antialiasing

  double highestPoint = flatRotatedPoints[indexs[0]][2];
  double lowestPoint = flatRotatedPoints[indexs[6*4*4-1]][2];
  double totalHeight = lowestPoint-highestPoint;
  double deltaHue = 255.0/(totalHeight*percentFilled); // delta hue for the number of leds filled

  static double hueOffset = 0; // hue offset
  double hueOffsetDelta = 3;

  for (size_t i = 0; i < 6*4*4; i++) {
    double height = flatRotatedPoints[i][2];
    if (height <= (totalHeight*percentFilled)-lowestPoint) {
      double hue = (height+lowestPoint)*deltaHue + hueOffset;

      // antialiasing (kinda)
      double vibrance = 255;
      if (height > (totalHeight*percentFilled)-lowestPoint-antialiasingHeight) {
        vibrance = (-255/antialiasingHeight)*(height-((totalHeight*percentFilled)-lowestPoint));
      }

      leds[i] = CHSV(hue, 255, vibrance);
    }
    else {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();

  // change hue offset
  if (hueOffset < 255) {
    hueOffset += hueOffsetDelta;
  }
  else {
    hueOffset -= 255;
  }
}

int sortByHeight(const void *cmp1, const void *cmp2) {
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  
  return flatRotatedPoints[a][2] - flatRotatedPoints[b][2];
}

void rotatePoint(double* point, double angle, double* rotationAxis, double* rotatedPoint) {
  double t = angle; 
  double ux = rotationAxis[0];
  double uy = rotationAxis[1];
  double uz = rotationAxis[2];
  BLA::Matrix<3,3> rotationMatrix;

  // cache rotation matrix
  static double prevT, prevUx, prevUy, prevUz;
  static BLA::Matrix<3,3> prevRotationMatrix;
  if (t == prevT && ux == prevUx && uy == prevUy && uz == prevUz) {
    rotationMatrix = prevRotationMatrix;
  }
  else {
    // rotationg angle t around axis u
    rotationMatrix = {
      cos(t)+pow(ux,2)*(1-cos(t)), ux*uy*(1-cos(t))-uz*sin(t),  ux*uz*(1-cos(t))+uy*sin(t),
      uy*ux*(1-cos(t))+uz*sin(t),  cos(t)+pow(uy,2)*(1-cos(t)), uy*uz*(1-cos(t))-ux*sin(t),
      uz*ux*(1-cos(t))-uy*sin(t),  uz*uy*(1-cos(t))+ux*sin(t),  cos(t)+pow(uz,2)*(1-cos(t))
    };

    prevT = t;
    prevUx = ux;
    prevUy = uy;
    prevUz = uz;
    prevRotationMatrix = rotationMatrix;
  }

  BLA::Matrix<3,1> pointMatrix = {point[0], point[1], point[2]};

  BLA::Matrix<3,1> rotatedPointMatrix = rotationMatrix * pointMatrix;
  rotatedPoint[0] = rotatedPointMatrix(0);
  rotatedPoint[1] = rotatedPointMatrix(1);
  rotatedPoint[2] = rotatedPointMatrix(2);
}

void gameOfLife() {
  delay(1000);
  /*
    6 2 1
      3
      4
      5 
  */
  static bool cells[18][14] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  };
  bool newCells[18][14];

  // calculate cells
  for (size_t i = 0; i < 18; i++) {
    for (size_t j = 0; j < 14; j++) {
      newCells[i][j] = isCellAlive(cells[i][j], 
        cells[i-1][j-1] + cells[i-1][j] + cells[i-1][j+1] + cells[i][j-1] + 
        cells[i][j+1] + cells[i+1][j-1] + cells[i+1][j] + cells[i+1][j+1]
      );
    }
  }

  // copy array
  for (size_t i = 0; i < 18; i++) {
    for (size_t j = 0; j < 14; j++) {
      cells[i][j] = newCells[i][j];
    }
  }

  // print cells
  for (size_t i = 0; i < 18; i++){
    for (size_t j = 0; j < 14; j++){
      Serial.print(cells[i][j]);
      Serial.print(" ");
    }
    Serial.print("\n");
  }

  // side 6
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 5*16] = cells[i+1][j+1] ? CRGB::Red : CRGB::Black;
    }
  }
  // side 2
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 1*16] = cells[i+1][j+5] ? CRGB::Red : CRGB::Black;
    }
  }
  // side 1
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 0*16] = cells[4-i][12-j] ? CRGB::Red : CRGB::Black;
    }
  }
  // side 3
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 2*16] = cells[8-i][j+5] ? CRGB::Red : CRGB::Black;
    }
  }
  // side 4
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 3*16] = cells[i+9][j+5] ? CRGB::Red : CRGB::Black;
    }
  }
  // side 5
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      leds[4*i+j + 4*16] = cells[16-i][j+5] ? CRGB::Red : CRGB::Black;
    }
  }
  FastLED.show();
  
}

bool isCellAlive(bool isCurrentlyAlive, int numOfAliveNeighbors) {
  // Any live cell with fewer than two live neighbours dies, as if by underpopulation.
  if (isCurrentlyAlive && numOfAliveNeighbors < 2) {
    return 0;
  }
  // Any live cell with two or three live neighbours lives on to the next generation.
  else if (isCurrentlyAlive && (numOfAliveNeighbors == 2 || numOfAliveNeighbors == 3)) {
    return 1;
  }
  // Any live cell with more than three live neighbours dies, as if by overpopulation.
  else if (isCurrentlyAlive && numOfAliveNeighbors > 3) {
    return 0;
  }
  // Any dead cell with exactly three live neighbours becomes a live cell, as if by reproduction.
  else if (!isCurrentlyAlive && numOfAliveNeighbors == 3) {
    return 1;
  }
}

double length(double* vector) {
  return sqrt(pow(vector[0],2)+pow(vector[1],2)+pow(vector[2],2));
}

void calibrate() {
  int dataPoints = 10;
  adxl.setAxisOffset(0, 0, 0);

  int minMaxValues[] = {-257, 253, -243, 271, -256, 235};

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

void getAngles(double* accels, double* angles) {
  // x
  if (fabs(accels[0]) < 0.05 && fabs(accels[2]) < 0.05) {
    angles[0] = 0;
  }
  else {
      angles[0] = atan(accels[0]/accels[2]) - PI;
      if (accels[2] < 0) {
        angles[0] += PI;
      }
      else if (accels[0] < 0) {
        angles[0] += PI*2;
      }
  }

  // y
  if (fabs(accels[1]) < 0.05 && fabs(accels[2]) < 0.05) {
    angles[1] = 0;
  }
  else {
    angles[1] = atan(accels[1]/accels[2]) - PI;
    if (accels[2] < 0) {
      angles[1] += PI;
    }
    else if (accels[1] < 0) {
      angles[1] += PI*2;
    }
  }

  // z
  if (fabs(accels[1]) < 0.05 && fabs(accels[0]) < 0.05) {
    angles[2] = 0;
  }
  else {
    angles[2] = atan(accels[1]/accels[0]) - PI;
    if (accels[0] < 0) {
      angles[2] += PI;
    }
    else if (accels[1] < 0) {
      angles[2] += PI*2;
    }
  }
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

void enterDeepSleep() {
  Serial.println("Entering Deep Sleep");
  
  FastLED.clear();
  FastLED.show();
  delay(30);
  
  adxl.setLowPower(true);
  ESP.deepSleep(0);
}

double clip(double n, double lower, double upper) {
  return max(lower, min(n, upper));
}

bool checkIsAbovePlane(double* normalVector, double* pointOnPlane, double* point) {
  double dotProduct = 
    normalVector[0]*(point[0]-pointOnPlane[0]) + 
    normalVector[1]*(point[1]-pointOnPlane[1]) + 
    normalVector[2]*(point[2]-pointOnPlane[2]); // normalVector⋅(pont-pointOnPlane)
  
  return dotProduct > 0;
}