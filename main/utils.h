#ifndef UTILS_H
#define UTILS_H

#include "config.h"

// ----------------------------
// 角度差分を計算するヘルパー関数
// ----------------------------
float angleDifference(float a, float b) {
  // 角度をラジアンに変換
  float diff = radians(a) - radians(b);
  // -πからπの範囲に正規化
  while (diff > PI) diff -= 2 * PI;
  while (diff < -PI) diff += 2 * PI;
  return degrees(diff);
}

// ----------------------------
// エンコーダの割り込み処理
// ----------------------------
void rightEncoderA_ISR() {
  bool stateA = digitalRead(right_OUT1);
  bool stateB = digitalRead(right_OUT2);
  if (stateA == stateB) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void rightEncoderB_ISR() {
  bool stateA = digitalRead(right_OUT1);
  bool stateB = digitalRead(right_OUT2);
  if (stateA != stateB) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void leftEncoderA_ISR() {
  bool stateA = digitalRead(left_OUT1);
  bool stateB = digitalRead(left_OUT2);
  if (stateA == stateB) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void leftEncoderB_ISR() {
  bool stateA = digitalRead(left_OUT1);
  bool stateB = digitalRead(left_OUT2);
  if (stateA != stateB) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

// ----------------------------
// モータ制御関数
// ----------------------------

void setMotorRight(float speed) {
  // speed: -255 ~ 255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(right_IN1, speed);
    digitalWrite(right_IN2, LOW);
  } else {
    analogWrite(right_IN1, -speed);
    digitalWrite(right_IN2, HIGH);
  }
}

void setMotorLeft(float speed) {
  // speed: -255 ~ 255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(left_IN1, speed);
    digitalWrite(left_IN2, LOW);
  } else {
    analogWrite(left_IN1, -speed);
    digitalWrite(left_IN2, HIGH);
  }
}

// ----------------------------
// PID制御関数
// ----------------------------
float pidControl(float setpoint, float measured, float &integral, float &prevError, float dt) {
  float error = angleDifference(setpoint, measured);
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
  return output;
}

float readFloatFromProgmem(const float* addr) {
  union {
    uint32_t asUint32;
    float asFloat;
  } data;
  data.asUint32 = pgm_read_dword_near(addr);
  return data.asFloat;
}


#endif