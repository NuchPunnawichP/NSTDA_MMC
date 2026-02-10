#define DEBUG
#include <SoftwareSerial.h>
#include <MPU9250.h>
#include <Wire.h>
// #include <VL6180X.h>
#include "Maze.h"
#include "Sensor.h"
#include "encoders.h"

SoftwareSerial BT(7, 8); // RX | TX (Arduino)

#define THRESHOLD 18
// #define WHEEL_DIAMETER 33.8 // 44mm
// #define WHEEL_DISTANCE 76 // 106mm
// #define ENCODER_PULSES_PER_REV 700 //190
// #define PI 3.14159265359

#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 9
#define MOTOR_RIGHT_FORWARD 10
#define MOTOR_RIGHT_BACKWARD 11
// #define ENC_LEFT_A 2
// #define ENC_LEFT_B 4
// #define ENC_RIGHT_A 3
// #define ENC_RIGHT_B 5

#define RightSensorAdd 0x20
#define FrontSensorAdd 0x22
#define LeftSensorAdd 0x24

// const float wheelCircumference = PI * WHEEL_DIAMETER; // mm
// const float pulsesPerMM = ENCODER_PULSES_PER_REV / wheelCircumference;
// const float degreesPerMM = 360.0 / (PI * WHEEL_DISTANCE);

// float robotAngle = 0.0;    // Current angle of the robot (degrees)

// volatile int leftEncoderCount = 0;
// volatile int rightEncoderCount = 0;

MPU9250 mpu;
MouseSensors sensors;
Encoders encoders; 
// VL6180X RightSensor;
// VL6180X FrontSensor;
// VL6180X LeftSensor;

int targetFront;

int thresholdFront;

int targetSide;
int thresholdSide;

MouseMaze<16,16> maze; // <rows,cols>

// ตัวแปร global flag ที่ใช้แจ้งให้ทราบว่าได้ครบเวลาที่จะอัปเดตแล้ว
volatile bool updateFlag = false;

// Timer2 ISR สำหรับเปรียบเทียบค่า (Compare Match A)
ISR(TIMER2_COMPA_vect) {
  updateFlag = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

  encoders.begin();
  sensors.configure();
  sensors.setupVL6180X();
  sensors.initialize();
  sensors.view();

   // *** Maze Solving ***

  // Start initial position (TEST: 2,0,NORTH | 3,3,WEST | 1,1,NORTH | 3,2,EAST | 1,5,SOUTH |  | 1,1,EAST | 3,3,NORTH)
  //สนามซ้อม
  maze.mouseRow = 10; // 15
  maze.mouseColumn = 0;
  maze.mouseHeading = NORTH;

  // Target position
  maze.targetRow = 3; //8
  maze.targetColumn = 7; //8

  // maze.mouseRow = 15;
  // maze.mouseColumn = 0;
  // maze.mouseHeading = NORTH;

  // // Target position
  // maze.targetRow = 8;
  // maze.targetColumn = 8;

  // Init walls
  //maze.addVirtualWalls();
  //maze.addVirtualWalls_test2();
  //maze.addWalls(NORTH);
  //maze.addWalls(EAST);
  //maze.addWalls(WEST);
  //maze.addWalls(SOUTH);
  //scanWalls();

  // Start solve
  maze.solveFloodFill();

  // Print maza
  maze.printMaze();
  // // Stepping through the maze!

  do {
    scanWalls();
    maze.solveFloodFill();
    
    #ifdef DEBUG
      maze.printMaze();
    #endif
    turnToNextMove();
    forwardProportional(164.5);
    Serial.println("Forward one cell");
    //sensors.sense();
   // while(Serial.read() != 'g');
    // maze.mouseRow += neighboringCells[maze.mouseHeading][0];
    // maze.mouseColumn += neighboringCells[maze.mouseHeading][1];

  
  }while(maze.values[maze.mouseRow][maze.mouseColumn] != 0);
 
   #ifdef DEBUG
  scanWalls();
  maze.solveFloodFill();
  maze.printMaze();
  #endif

  delay(2000); // delay For Move Back
  spinTurnRight(180.0, 1.8, 0.1);

  maze.mouseRow = 3; // 8
  maze.mouseColumn = 7; // 8
  maze.mouseHeading = EAST;

  // Target position
  maze.targetRow = 10; // 15
  maze.targetColumn = 0;

  do
  {
    scanWalls();
    maze.solveFloodFill();
    maze.printMaze();
   // delay(10); // temporal
    turnToNextMove();
    forwardProportional(164.5);
  }while(maze.values[maze.mouseRow][maze.mouseColumn] != 0);

  turnToNextMove();
  
  #ifdef DEBUG
  scanWalls();
  maze.solveFloodFill();
  maze.printMaze();
  #endif

}

void loop() 
{

}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(MOTOR_LEFT_FORWARD, leftSpeed);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
  } else {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, -leftSpeed);
  }

  if (rightSpeed > 0) {
    analogWrite(MOTOR_RIGHT_FORWARD, rightSpeed);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  } else {
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, -rightSpeed);
  }
}

void scanWalls(){
  int front_threshold = 85;
  int left_threshold = 100;
  int right_threshold = 100;

  sensors.initialize();
  
  if(sensors.frontleft < front_threshold)  
    maze.addWalls(maze.mouseHeading);
    
  if(sensors.right < right_threshold)
    maze.addWalls((maze.mouseHeading + 1) % 4);

  if(sensors.left < left_threshold)
    maze.addWalls((maze.mouseHeading - 1 + 4) % 4);
    
}

void turnToNextMove(){
  
  byte desiredHeading = maze.findNextMove();
  int difference = maze.mouseHeading - desiredHeading;

    if(difference == 1 || difference == -3){
      spinTurnLeft(87.0, 1.8, 0.2);
      //Serial.println("TurnLeft");
      sensors.initialize();
    }else if(difference == 3 || difference == -1){
      spinTurnRight(87.0, 1.8, 0.2);
      //Serial.println("TurnRight");
      sensors.initialize();
    }else if(difference == 2 || difference == -2){
      //motors.turn((random(2))?LEFT:RIGHT, 180);
      sensors.sense();
      if(sensors.left > sensors.right)
        //Serial.println("TurnLeft 180");
        spinTurnLeft(180.0, 1.8, 0.2);
      else
        //Serial.println("TurnRight 180");
        spinTurnRight(180.0, 1.8, 0.2);
      //(random(2) == 0) ? spinTurnRight(180.0, 1.8, 0.2) : spinTurnLeft(180.0, 1.8, 0.4);
      sensors.initialize();
    }

    maze.mouseHeading = desiredHeading;

}

byte state(){
  int front_threshold = 25;
  int left_threshold = 70;
  int right_threshold = 70;
  byte event = 0;




  // Serial.print("Front: ");
  // Serial.print(distanceFront);
  // Serial.print(" | Left: ");
  // Serial.print(distanceLeft);
  // Serial.print(" | Right: ");
  // Serial.println(distanceRight);

  if(sensors.frontleft > front_threshold)
    event += 1;
  if(sensors.left > left_threshold)
    event += 2;
  if(sensors.right > right_threshold)
    event += 4;

  return event;
}

void calibrate(){
  
  sensors.initialize();
  targetSide = (sensors.left + sensors.right)/2;
  
  spinTurnRight(90, 1.8,0.1);
  sensors.initialize();
  targetFront = sensors.frontleft;
  
  thresholdSide = (targetSide + sensors.left * 2)/3;  // * 2)/3 : Si a veces no detecta las paredes

  spinTurnLeft90(1.8,0.1);
  sensors.initialize();

  thresholdFront = (targetFront + sensors.frontleft * 3)/4;  // * 2)/3 : Si a veces no detecta las paredes
  spinTurnRight(90, 1.8,0.1);

}

void forwardProportional(float targetDistanceMM) {
    const int baseSpeed = 70;
    const float Kp = 0.01; // 0.01
    const float Ki = 0.05; // 0.05
    const float Kd = 0.25; // 0.25
    const int desiredDistanceWall = 45;
    const int stopDistanceFront = 30;

    encoders.resetEncoders();

    float distanceTravelled = 0.0;
    float accumulatedError = 0.0;
    float previousError = 0.0;
    unsigned long previousTime = millis();

    while (distanceTravelled < targetDistanceMM) {
        sensors.sense();
       // sensors.view();

        if (sensors.frontright < stopDistanceFront) {
            Serial.println("Obstacle detected! Stopping.");
            break;
        }

        // คำนวณ error สำหรับการรักษาระยะห่าง
        float error = (sensors.left - sensors.right) * 1.5;

        // **เพิ่มการปรับการหมุนเมื่อหุ่นยนต์ใกล้กำแพง**
        float rotationError = 0.0;
        if (sensors.left < desiredDistanceWall || sensors.right < desiredDistanceWall) {
            // เมื่อหุ่นยนต์ใกล้กำแพงมากเกินไป
            rotationError = (sensors.left - sensors.right);
        }

        // สะสมค่าผิดพลาดเพื่อควบคุมระยะห่าง
        accumulatedError += error;
        accumulatedError = constrain(accumulatedError, -100, 100);

        // คำนวณ Derivative Term
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - previousTime) / 1000.0;
        if (deltaTime < 0.001) deltaTime = 0.001;

        float derivative = (error - previousError) / deltaTime;
        derivative = constrain(derivative, -50, 50);

        // คำนวณ Correction โดยใช้ PID
        int correction = (int)(Kp * error + Ki * accumulatedError + Kd * derivative);

        // **เพิ่มการปรับการหมุน (rotation)**
        correction += rotationError * 0.5; // ปรับค่าของ rotationError เพิ่มขึ้นหากห่างจากกำแพง

        // **จำกัดค่า Correction ไม่ให้สูงเกินไป**
        correction = constrain(correction, -50, 50);

        // คำนวณการปรับสมดุลระหว่างล้อซ้ายและขวา
        float KpBalance = 0.8;
        float balanceError = encoders.leftEncoderCount - encoders.rightEncoderCount;
        balanceError = constrain(balanceError, -10, 10);
        float balanceSignal = KpBalance * balanceError;

        // ปรับความเร็วของล้อซ้ายและขวา
        int leftSpeed = baseSpeed + correction - balanceSignal;
        int rightSpeed = baseSpeed + correction + balanceSignal;

        // จำกัดค่า PWM ให้อยู่ในช่วง 0 ถึง 255
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);

        setMotorSpeeds(leftSpeed, rightSpeed);

        // คำนวณระยะทางที่เดินทางได้
        distanceTravelled = encoders.averageEncoderCount() / encoders.pulsesPerMM;

        // อัปเดตค่า previousError และเวลา
        previousError = error;
        previousTime = currentTime;

        delay(20);  // หน่วงเล็กน้อยเพื่อความเสถียร
    }

    setMotorSpeeds(0, 0);

    maze.mouseRow += neighboringCells[maze.mouseHeading][0];
    maze.mouseColumn += neighboringCells[maze.mouseHeading][1];
} // this OK ถ้าวางหุ่นยนต์ตรง แต่มีปัญหา ถ้าเดินชิดกำแพงดานใดด้านหนึ่งจะช้า

//motion control
void moveForwardPD(float targetDistanceMM, float Kp, float Kd, float KpBalance, int minPWM) {
  encoders.resetEncoders();

  float error = 0.0;
  float lastError = 0.0;
  float derivative = 0.0;
  float controlSignal = 0.0;
  float distanceTravelled = 0.0;

  int baseSpeed = 80; // Base motor speed (adjust as needed)

  while (distanceTravelled < targetDistanceMM) {

    // Serial.print("Encoder Left: ");
    // Serial.print(leftEncoderCount);
    // Serial.print("| EncoderRight: ");
    // Serial.println(rightEncoderCount);
    
    
    // คำนวณระยะทางที่เดินทางได้
    distanceTravelled = encoders.averageEncoderCount() / encoders.pulsesPerMM;

    // คำนวณค่า error
    error = targetDistanceMM - distanceTravelled;

    // คำนวณค่า derivative
    derivative = error - lastError;

    // คำนวณสัญญาณควบคุม PD
    controlSignal = (Kp * error) + (Kd * derivative);

    // คำนวณความสมดุลของล้อซ้ายและขวา
    float balanceError = encoders.leftEncoderCount - encoders.rightEncoderCount;
    float balanceSignal = KpBalance * balanceError;

    // ปรับความเร็วล้อซ้ายและขวา
    int leftSpeed = baseSpeed + controlSignal - balanceSignal;
    int rightSpeed = baseSpeed + controlSignal + balanceSignal;

    // ตรวจสอบและปรับค่า PWM ให้อยู่ในช่วงที่มอเตอร์เคลื่อนที่ได้
    if (abs(leftSpeed) > 0 && abs(leftSpeed) < minPWM) {
      leftSpeed = (leftSpeed > 0) ? minPWM : -minPWM;
    }
    if (abs(rightSpeed) > 0 && abs(rightSpeed) < minPWM) {
      rightSpeed = (rightSpeed > 0) ? minPWM : -minPWM;
    }

    // จำกัดค่า PWM อยู่ในช่วง 0 ถึง 255
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Serial.print("Left speed: ");
    // Serial.print(leftSpeed);
    // Serial.print("| Right speed: ");
    // Serial.println(rightSpeed);

    setMotorSpeeds(leftSpeed, rightSpeed);

    // บันทึกค่า error ล่าสุด
    lastError = error;

    // Delay เล็กน้อยเพื่อความเสถียร
    delay(10);
  }

  // หยุดมอเตอร์เมื่อถึงเป้าหมาย
  setMotorSpeeds(0, 0);

  // Maze navigate
  maze.mouseRow += neighboringCells[maze.mouseHeading][0];
  maze.mouseColumn += neighboringCells[maze.mouseHeading][1];
}

void spinTurnLeft90(float Kp, float Kd) {
  // robotAngle "เพิ่มขึ้น" เมื่อหมุนซ้าย
  // => หมุนซ้าย 90 องศา = robotAngle + 90
  float targetAngle = encoders.robotAngle + 90.0;
  if (targetAngle >= 360.0) {
    targetAngle -= 360.0;
  }

  float error = 0.0, lastError = 0.0, derivative = 0.0;
  float output = 0.0;

  encoders.resetEncoders();
  while (true) {
    encoders.updateRobotState();  // คำนวณ robotAngle ใหม่

    // หมุนซ้าย = อยากให้ angle ไปเพิ่มถึง targetAngle
    error = targetAngle - encoders.robotAngle;

    // Normalize error ให้อยู่ในช่วง -180..180 
    if (error > 180.0)  error -= 360.0;
    if (error < -180.0) error += 360.0;

    // เผื่อหยุดใกล้ ๆ เป้าหมาย
    if (fabs(error) < 1.0) {
      break;
    }

    derivative = error - lastError;
    output = (Kp * error) + (Kd * derivative);

    // ล้อซ้ายถอยหลัง (-output), ล้อขวาเดินหน้า (+output)
    int pwmLeft  = constrain((int)(-output), -255, 255);
    int pwmRight = constrain((int)(+output), -255, 255);

    // บังคับ minPWM
    int minPWM = 60;
    if (abs(pwmLeft) < minPWM && pwmLeft != 0) {
      pwmLeft = (pwmLeft > 0) ? minPWM : -minPWM;
    }
    if (abs(pwmRight) < minPWM && pwmRight != 0) {
      pwmRight = (pwmRight > 0) ? minPWM : -minPWM;
    }

    setMotorSpeeds(pwmLeft, pwmRight);
    lastError = error;
    delay(10);
  }
  setMotorSpeeds(0, 0);
}

void spinTurnRight90(float Kp, float Kd) {
  float targetAngle = encoders.robotAngle - 90.0;
  if (targetAngle < 0)   targetAngle += 360;
  //if (targetAngle >= 360) targetAngle -= 360;

  float error = 0.0, lastError = 0.0, output = 0.0;

  while (true) {
    encoders.updateRobotState(); // อ่านค่า Encoder เพื่อคำนวณ robotAngle
    error = targetAngle - encoders.robotAngle;
    // Normalize error -180..180
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < 1.0) break; 

    float derivative = error - lastError;
    output = (Kp * error) + (Kd * derivative);

    // บังคับให้อยู่ในช่วง PWM
    int pwmLeft  = constrain((int)(-output), -255, 255);
    int pwmRight = constrain((int)(+output), -255, 255);

    // บังคับ minPWM หากต้องการ
    int minPWM = 50;  // ตัวอย่าง
    if (abs(pwmLeft) < minPWM && pwmLeft != 0) {
      pwmLeft = (pwmLeft > 0) ? minPWM : -minPWM;
    }
    if (abs(pwmRight) < minPWM && pwmRight != 0) {
      pwmRight = (pwmRight > 0) ? minPWM : -minPWM;
    }

    // Serial.print(" pwmLeft");
    // Serial.print(pwmLeft);
    // Serial.print(" | pwmRight");
    // Serial.println(pwmRight);

    setMotorSpeeds(pwmLeft, pwmRight);
    lastError = error;
    delay(10);
  }
  setMotorSpeeds(0, 0);
}

void spinTurnRight(float degrees, float Kp, float Kd) {
  // 1) กำหนดมุมเป้าหมาย
  float targetAngle = encoders.robotAngle - degrees;
  if (targetAngle < 0)   targetAngle += 360;
  if (targetAngle >= 360) targetAngle -= 360;

  // 2) ตัวแปรสำหรับ PD
  float error = 0.0, lastError = 0.0, output = 0.0;

  while (true) {
    encoders.updateRobotState(); // อ่านค่า Encoder เพื่อคำนวณ robotAngle

    #ifdef DEBUG
    // Serial.print("Omega: ");
    // Serial.print(encoders.robot_omega());
    // Serial.print("\t target: ");
    // Serial.print(targetAngle);
    // Serial.print("\t Current: ");
    // Serial.println(encoders.robotAngle);
    #endif
    // 3) คำนวณค่าข้อผิดพลาด
    error = targetAngle - encoders.robotAngle;

    // 4) Normalize error ให้อยู่ในช่วง -180..180
    if (error > 180)  error -= 360;
    if (error < -180) error += 360;

    // 5) เงื่อนไขหยุด
    if (fabs(error) < 1.0) {
      break;
    }

    // 6) คำนวณ PD
    float derivative = error - lastError;
    output = (Kp * error) + (Kd * derivative);

    // 7) กำหนด PWM และ minPWM
    int pwmLeft  = constrain((int)(-output), -255, 255);
    int pwmRight = constrain((int)(+output), -255, 255);

    int minPWM = 50;  // สามารถปรับได้
    if (abs(pwmLeft) < minPWM && pwmLeft != 0) {
      pwmLeft = (pwmLeft > 0) ? minPWM : -minPWM;
    }
    if (abs(pwmRight) < minPWM && pwmRight != 0) {
      pwmRight = (pwmRight > 0) ? minPWM : -minPWM;
    }

    // 8) ขับมอเตอร์
    setMotorSpeeds(pwmLeft, pwmRight);

    lastError = error;
    delay(10);
  }

  // หยุดมอเตอร์
  setMotorSpeeds(0, 0);
}

void spinTurnLeft(float degrees, float Kp, float Kd) {
  // รีเซ็ต encoder ก่อนคำนวณ targetAngle
  encoders.resetEncoders();
  float targetAngle = encoders.robotAngle + degrees;
  if (targetAngle < 0)   targetAngle += 360;
  if (targetAngle >= 360) targetAngle -= 360;

  float error = 0.0, lastError = 0.0, derivative = 0.0;
  float output = 0.0;

  while (true) {
    encoders.updateRobotState();  // อัปเดต robotAngle ใหม่

    #ifdef DEBUG
    // Serial.print("Omega: ");
    // Serial.print(encoders.robot_omega());
    // Serial.print("\t target: ");
    // Serial.print(targetAngle);
    // Serial.print("\t Current: ");
    // Serial.println(encoders.robotAngle);
    #endif
    // คำนวณ error สำหรับการเลี้ยวซ้าย (เราต้องการให้ robotAngle เพิ่มขึ้น)
    error = targetAngle - encoders.robotAngle;

    // Normalize error ให้อยู่ในช่วง -180 ถึง 180
    if (error > 180.0)  error -= 360.0;
    if (error < -180.0) error += 360.0;

    // หยุดเมื่อ error ใกล้ 0
    if (fabs(error) < 1.0) {
      break;
    }

    derivative = error - lastError;
    output = (Kp * error) + (Kd * derivative);

    // กำหนดความเร็วให้มอเตอร์: ซ้ายถอยหลัง (-output), ขวาเดินหน้า (+output)
    int pwmLeft  = constrain((int)(-output), -255, 255);
    int pwmRight = constrain((int)(+output), -255, 255);

    // กำหนดค่า minPWM หาก output มีค่าน้อยเกินไป
    int minPWM = 50;
    if (abs(pwmLeft) < minPWM && pwmLeft != 0) {
      pwmLeft = (pwmLeft > 0) ? minPWM : -minPWM;
    }
    if (abs(pwmRight) < minPWM && pwmRight != 0) {
      pwmRight = (pwmRight > 0) ? minPWM : -minPWM;
    }

    setMotorSpeeds(pwmLeft, pwmRight);
    lastError = error;
    delay(10);
  }
  setMotorSpeeds(0, 0);
}





