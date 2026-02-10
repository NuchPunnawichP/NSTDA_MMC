#include <Arduino.h>
#include <VL6180X.h>

#define RightSensorAdd 0x20
#define FrontLeftSensorAdd 0x22
#define FrontRightSensorAdd 0x24
#define LeftSensorAdd 0x26

VL6180X RightSensor;
VL6180X FrontLeftSensor;
VL6180X FrontRightSensor;
VL6180X LeftSensor;

//template<byte leftEmitter, byte leftDetector, byte frontEmitter, byte frontDetector, byte rightEmitter, byte rightDetector>

class MouseSensors {

private:
  // Used to store sensor values
  int distanceFrontLeft = 0;
  int distanceFrontRight = 0;
  int distanceLeft = 0;
  int distanceRight = 0;
  // Variables used for smoothing
  int leftTotal;
  int frontLeftTotal;
  int frontRightTotal;
  int rightTotal;
  static const byte numReadings = 10;
  byte index;
  int leftReadings[numReadings];
  int frontLeftReadings[numReadings];
  int frontRightReadings[numReadings];
  int rightReadings[numReadings];

  int leftSmoothed;
  int frontLeftSmoothed;
  int frontRightSmoothed;
  int rightSmoothed;

public:

  int left;
  int frontleft;
  int frontright;
  int right;

  void setupVL6180X() 
  {
    PORTC |= (1 << 0);
    delay(1);
    RightSensor.init();
    RightSensor.configureDefault();
    RightSensor.setAddress(RightSensorAdd);
    RightSensor.setScaling(2);
    RightSensor.setTimeout(50);

    PORTC |= (1 << 1);
    delay(1);
    FrontLeftSensor.init();
    FrontLeftSensor.configureDefault();
    FrontLeftSensor.setAddress(FrontLeftSensorAdd);
    FrontLeftSensor.setScaling(2);
    FrontLeftSensor.setTimeout(50);

    PORTC |= (1 << 2);
    delay(1);
    LeftSensor.init();
    LeftSensor.configureDefault();
    LeftSensor.setAddress(LeftSensorAdd);
    LeftSensor.setScaling(2);
    LeftSensor.setTimeout(50);

    PORTC |= (1 << 3);
    delay(1);
    FrontRightSensor.init();
    FrontRightSensor.configureDefault();
    FrontRightSensor.setAddress(FrontRightSensorAdd);
    FrontRightSensor.setScaling(2);
    FrontRightSensor.setTimeout(50);
  }

  void configure()
  {
    DDRC = 0x0F; // Set PIN A0-A3 for OUTPUT to control SHUT PIN

    Wire.begin();
    delay(20);
  }                                                                                

  void sense() {
    distanceRight = ReadRightSensor() + 16; //+26 for error right sensor 
    distanceFrontLeft = ReadFrontLeftSensor() ;
    distanceFrontRight = ReadFrontRightSensor() - 14;
    distanceLeft = ReadLeftSensor() - 5; // -10 for error left sensor
    

    // Smoothing
    leftTotal -= leftReadings[index];
    frontRightTotal -= frontRightReadings[index];
    frontLeftTotal -= frontLeftReadings[index];
    rightTotal -= rightReadings[index];

    leftReadings[index] = distanceLeft;
    frontLeftReadings[index] = distanceFrontLeft;
    frontRightReadings[index] = distanceFrontRight;
    rightReadings[index] = distanceRight;

    leftTotal += leftReadings[index];
    frontLeftTotal += frontLeftReadings[index];
    frontRightTotal += frontRightReadings[index];
    rightTotal += rightReadings[index];

    leftSmoothed = leftTotal / numReadings;
    frontLeftSmoothed = frontLeftTotal / numReadings;
    frontRightSmoothed = frontRightTotal / numReadings;
    rightSmoothed = rightTotal / numReadings;

    left = leftSmoothed;
    frontleft = frontLeftSmoothed;
    frontright = frontRightSmoothed;
    right = rightSmoothed;

    index += 1;

    if (index >= numReadings) {
      index = 0;
    }

    //view();
  }

  void initialize() {
    for (byte i = 0; i < numReadings; i++) {
      sense();
    }
  }

  void halfInitialize() {
    for (byte i = 0; i < numReadings / 2; i++) {
      sense();
    }
  }

  void view() {
    Serial.print("Sensors: ");

    Serial.print(distanceLeft);
    Serial.print("\t L -> ");
    Serial.print(left);

    Serial.print("\t | \t");

    Serial.print(distanceFrontLeft);
    Serial.print("\t FL -> ");
    Serial.print(frontleft);

    Serial.print("\t | \t");

    Serial.print(distanceFrontRight);
    Serial.print("\t FR -> ");
    Serial.print(frontright);

    Serial.print("\t | \t");

    Serial.print(distanceRight);
    Serial.print("\t R -> ");
    Serial.println(right);
  }

  int ReadRightSensor() {
  PORTC |= (1 << 0);
  delay(1);
  return RightSensor.readRangeSingleMillimeters();
  }

  int ReadFrontLeftSensor() {
    PORTC |= (1 << 1);
    delay(1);
    return FrontLeftSensor.readRangeSingleMillimeters();
  }

  int ReadFrontRightSensor() {
    PORTC |= (1 << 3);
    delay(1);
    return FrontRightSensor.readRangeSingleMillimeters();
  }

  int ReadLeftSensor() {
    PORTC |= (1 << 2);
    delay(1);
    return LeftSensor.readRangeSingleMillimeters();
  }
};