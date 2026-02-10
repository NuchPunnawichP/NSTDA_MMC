#pragma once
#include <Arduino.h>
#include <stdint.h>

#define WHEEL_DIAMETER 33.8      // mm
#define WHEEL_DISTANCE 76        // mm
#define ENCODER_PULSES_PER_REV 700
#define PI 3.14159265359

#define ENC_LEFT_A 2
#define ENC_LEFT_B 4
#define ENC_RIGHT_A 3
#define ENC_RIGHT_B 5

// Forward declaration of the class and global instance
class Encoders;
extern Encoders encoders;  // defined in your main file

// Declare free-function ISRs.
void leftEncoderISR();
void rightEncoderISR();

class Encoders {
  private:
    // Private variables would be placed here.
    float m_robot_distance = 0.0;
    float m_robot_speed = 0.0;    // ความเร็ว (mm/s)
    float m_robot_omega   = 0.0;  // ความเร็วเชิงมุม (deg/s)
    float m_robot_rotation_change = 0.0; // การเปลี่ยนแปลงเชิงมุมสะสม (deg)
    float m_robot_fwd_change = 0.0;      // การเปลี่ยนแปลงการเคลื่อนที่ไปข้างหน้าในรอบล่าสุด (mm/s/s)
    unsigned long lastUpdateTime = 0; // เวลาที่อัปเดตล่าสุด (millis)
  public:
    // Calculated constants
    const float wheelCircumference = PI * WHEEL_DIAMETER; // mm
    const float pulsesPerMM = ENCODER_PULSES_PER_REV / wheelCircumference;
    const float degreesPerMM = 360.0 / (PI * WHEEL_DISTANCE);

    // State variables
    float robotAngle = 0.0;    // Robot's current angle (degrees)
    volatile int leftEncoderCount = 0;
    volatile int rightEncoderCount = 0;
  
  Encoders() {
    resetEncoders();
    lastUpdateTime = millis();  // กำหนดเวลาเริ่มต้น
  }

  void begin() {
    pinMode(ENC_LEFT_A, INPUT);
    pinMode(ENC_LEFT_B, INPUT);
    pinMode(ENC_RIGHT_A, INPUT);
    pinMode(ENC_RIGHT_B, INPUT);

    // Use the free functions as ISR callbacks.
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), ::leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), ::rightEncoderISR, RISING);
  }

  void resetEncoders() {
    noInterrupts();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    m_robot_distance = 0.0;
    m_robot_speed = 0.0;
    m_robot_omega = 0.0;
    m_robot_rotation_change = 0.0;
    m_robot_fwd_change = 0.0;
    interrupts();

    lastUpdateTime = millis();
  }

  int averageEncoderCount() {
    noInterrupts();
    int avg = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
    interrupts();
    return avg;
  }

  void updateRobotState() {
    // คำนวณช่วงเวลาที่ผ่านไป (dt) ในหน่วยวินาที
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;  // แปลงจาก millis เป็นวินาที
    lastUpdateTime = currentTime;

    int leftDelta = 0;
    int rightDelta = 0;

    // อ่านค่า encoder counts อย่างปลอดภัย
    noInterrupts();
    leftDelta = leftEncoderCount;
    rightDelta = rightEncoderCount;
    // รีเซ็ต encoder counts หลังจากอ่านค่าแล้ว
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    interrupts();

    // คำนวณระยะทางที่แต่ละล้อเคลื่อนที่ (mm)
    float leftDistance = leftDelta / pulsesPerMM;
    float rightDistance = rightDelta / pulsesPerMM;
    
    // คำนวณระยะทางเฉลี่ยที่หุ่นยนต์เดินในช่วงเวลานี้ (mm)
    float distance = (leftDistance + rightDistance) / 2.0;
    
    // คำนวณความเร็วเชิงเส้น (mm/s)
    float speed = (dt > 0) ? (distance / dt) : 0.0;

    // อัปเดตรายการค่าระยะทาง, ความเร็ว และการเปลี่ยนแปลงการเคลื่อนที่ไปข้างหน้า
    noInterrupts();
    m_robot_distance += distance;
    m_robot_speed = speed;
    m_robot_fwd_change = distance;  // robot_fwd_change คือระยะทางที่เพิ่มขึ้นในรอบนี้
    interrupts();

    // คำนวณการเปลี่ยนแปลงมุม (deltaAngle) ในหน่วยองศา
    float deltaAngle = ((rightDistance - leftDistance) / WHEEL_DISTANCE) * (180.0 / PI);
    
    // คำนวณความเร็วเชิงมุม (omega) ในหน่วย deg/s
    float omega = (dt > 0) ? (deltaAngle / dt) : 0.0;
    
    // อัปเดตรายการค่า omega
    noInterrupts();
    m_robot_omega = omega;
    interrupts();

    // อัปเดตมุมหุ่นยนต์ (heading) และ normalize ให้อยู่ในช่วง 0-360
    robotAngle += deltaAngle;
    if (robotAngle < 0) robotAngle += 360;
    if (robotAngle >= 360) robotAngle -= 360;

    // สะสมการเปลี่ยนแปลงเชิงมุม (rotation change)
    noInterrupts();
    m_robot_rotation_change += deltaAngle;
    interrupts();
  }

  // Convenience methods to safely access encoder data (using interrupts disable/enable)
  float robot_distance() {
    float distance;
    noInterrupts();
    distance = m_robot_distance;
    interrupts();
    return distance;
  }

  float robot_speed() {
    float speed;
    noInterrupts();
    speed = m_robot_speed;
    interrupts();
    return speed;
  }

  float robot_omega() {
    float omega;
    noInterrupts();
    omega = m_robot_omega;
    interrupts();
    return omega;
  }

  float robot_fwd_change() {
    float fwd_change;
    noInterrupts();
    fwd_change = m_robot_fwd_change;
    interrupts();
    return fwd_change;
  }

  float robot_rot_change() {
    float rot_change;
    noInterrupts();
    rot_change = m_robot_rotation_change;
    interrupts();
    return rot_change;
  }

  float robot_angle_value() {
    float angle;
    noInterrupts();
    angle = robotAngle;
    interrupts();
    return angle;
  }

 
};

// --- Free Function ISR Definitions ---

// These functions run in interrupt context and update the global encoders instance.
inline void leftEncoderISR() {
  // For example, read the state of pin 4 (ENC_LEFT_B) via PIND.
  // Adjust the count based on the direction.
  encoders.leftEncoderCount += (PIND & (1 << PIND4)) ? -1 : 1;
}

inline void rightEncoderISR() {
  // Similarly, use pin 5 (ENC_RIGHT_B) for direction.
  encoders.rightEncoderCount += (PIND & (1 << PIND5)) ? 1 : -1;
}
