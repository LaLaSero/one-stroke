#include "config.h"
#include "utils.h"

float normalizeAngle(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }
  while (angle < -180)
  {
    angle += 360;
  }
  return angle;
}

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

void loop() {
  // 現在時間を取得
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= controlInterval)
  {
    float dt = (currentTime - prevTime) / 1000.0; // 秒に変換

    // 配列から目標位置を更新
    x_target = readFloatFromProgmem(x_targets + targetIndex);
    y_target = readFloatFromProgmem(y_targets + targetIndex);

    targetIndex++;
    if (targetIndex >= numPoints) {
      targetIndex = numPoints - 1;
    }

    // 逆運動学の計算（左アーム）
    bool leftIK = inverseKinematicsLeft(x_target, y_target, theta_current_left, theta_target_left);

    // 逆運動学の計算（右アーム）
    bool rightIK = inverseKinematicsRight(x_target, y_target, theta_current_right, theta_target_right);

    if (!leftIK || !rightIK) {
      Serial.println("Inverse Kinematics solution not found for the given position.");
      setMotorRight(0);
      setMotorLeft(0);
    } 
    else
    {
      // エンコーダから現在の角度を安全に計算
      noInterrupts();
      long rightCount = rightEncoderCount;
      long leftCount = leftEncoderCount;
      interrupts();

      theta_current_right = (rightCount / (float)CPR) * 360.0 / GearRatio;
      theta_current_left = (leftCount / (float)CPR) * 360.0 / GearRatio;

      theta_current_right = normalizeAngle(theta_current_right);
      theta_current_left = normalizeAngle(theta_current_left);
      
      // PID制御（右アーム）
      float controlSignal_right = pidControl(theta_target_right, theta_current_right, integral_right, prevError_right, dt);

      // PID制御（左アーム）
      float controlSignal_left = pidControl(theta_target_left, theta_current_left, integral_left, prevError_left, dt);

      // モータに制御信号を送信
      setMotorRight(controlSignal_right);
      setMotorLeft(controlSignal_left);

      // デバッグ用シリアル出力
      // Serial.print("Theta Target Right: "); Serial.print(theta_target_right);
      // Serial.print(" | Theta Current Right: "); Serial.print(theta_current_right);
      // Serial.print(" | Control Right: "); Serial.print(controlSignal_right);
      // Serial.print(" || Theta Target Left: "); Serial.print(theta_target_left);
      // Serial.print(" | Theta Current Left: "); Serial.print(theta_current_left);
      // Serial.print(" | Control Left: "); Serial.println(controlSignal_left);
    }

    // 時間を更新
    prevTime = currentTime;
  }
}


