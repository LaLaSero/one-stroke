#include "config.h"
#include "utils.h"
#include "reverse_kinematics.h"

void setup() {
  // シリアル通信の初期化
  Serial.begin(9600);

  // モータ制御ピンを出力に設定
  pinMode(right_IN1, OUTPUT);
  pinMode(right_IN2, OUTPUT);
  pinMode(left_IN1, OUTPUT);
  pinMode(left_IN2, OUTPUT);

  // エンコーダーのピンを入力に設定
  pinMode(right_OUT1, INPUT_PULLUP);
  pinMode(right_OUT2, INPUT_PULLUP);
  pinMode(left_OUT1, INPUT_PULLUP);
  pinMode(left_OUT2, INPUT_PULLUP);

  // エンコーダーの割り込み設定
  attachInterrupt(digitalPinToInterrupt(right_OUT1), rightEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_OUT2), rightEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_OUT1), leftEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_OUT2), leftEncoderB_ISR, CHANGE);

  // 初期時間を取得
  noInterrupts();
  rightEncoderCount = 0;
  leftEncoderCount = 0;
  interrupts();
  prevTime = millis();
}

int targetIndex = 0;
void loop()
{
  unsigned long currentTime = millis();

  // Target Update
  if (currentTime - prevTimeTargetUpdate >= targetUpdateInterval)
  {
    x_target = readFloatFromProgmem(x_targets + targetIndex);
    y_target = readFloatFromProgmem(y_targets + targetIndex);
    targetIndex++;
    if (targetIndex >= numPoints)
    {
      targetIndex = numPoints - 1;
    }

    prevTimeTargetUpdate = currentTime;
  }

  // Control Loop
  if (currentTime - prevTimeControlLoop >= controlLoopInterval) {
    float dt = (currentTime - prevTimeControlLoop) / 1000.0;

    // Perform Inverse Kinematics
    bool leftIK = inverseKinematicsLeft(x_target, y_target, theta_current_left, theta_target_left);
    bool rightIK = inverseKinematicsRight(x_target, y_target, theta_current_right, theta_target_right);

    if (!leftIK || !rightIK)
    {
      Serial.println("Inverse Kinematics solution not found for the given position.");
      setMotorRight(0);
      setMotorLeft(0);
    }
    else
    {
      noInterrupts();
      long rightCount = rightEncoderCount;
      long leftCount = leftEncoderCount;
      interrupts();

      theta_current_right = (rightCount / (float)CPR) * 360.0 / GearRatio;
      theta_current_left = (leftCount / (float)CPR) * 360.0 / GearRatio;

      float controlSignal_right = pidControl(theta_target_right, theta_current_right, integral_right, prevError_right, dt);
      float controlSignal_left = pidControl(theta_target_left, theta_current_left, integral_left, prevError_left, dt);

      setMotorRight(controlSignal_right);
      setMotorLeft(controlSignal_left);

      Serial.print("controlSignal_right: "); Serial.println(controlSignal_right);
      Serial.print("controlSignal_left: "); Serial.println(controlSignal_left);
    }
    prevTimeControlLoop = currentTime;
  }
}