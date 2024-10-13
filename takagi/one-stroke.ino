#include <Arduino.h>

// ----------------------------
// ハードウェアの設定
// ----------------------------

// モータ制御ピン（右側）
const int right_IN1 = 6;
const int right_IN2 = 7;
const int right_OUT1 = 18; // エンコーダーA
const int right_OUT2 = 19; // エンコーダーB

// モータ制御ピン（左側）
const int left_IN1 = 12;
const int left_IN2 = 13;
const int left_OUT1 = 20; // エンコーダーA
const int left_OUT2 = 21; // エンコーダーB

// エンコーダーの設定
const int CPR = 1000;        // エンコーダーのカウント数（例）
const float GearRatio = 1.0; // ギア比（例）

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount = 0;

// PID制御用のパラメータ
float Kp = 8.0;
float Ki = 0.0;
float Kd = 0.2;

// PID制御用の変数（右側）
float integral_right = 0;
float prevError_right = 0;

// PID制御用の変数（左側）
float integral_left = 0;
float prevError_left = 0;

// 制御周期
unsigned long prevTime = 0;
const unsigned long controlInterval = 10; // ミリ秒

// 目標位置（ペン先の座標）
float x_target = 150.0; // 単位：mm
float y_target = 50.0;  // 単位：mm

// 現在のモーター角度（degrees）
float theta_current_right = 0.0;
float theta_current_left = 0.0;

// モーターの目標角度（degrees）
float theta_target_right = 0.0;
float theta_target_left = 0.0;

// リンクの長さ
const float l0 = 200.0; // モーター間の距離（mm）
const float l1 = 100.0; // 第1リンクの長さ（mm）
const float l2 = 150.0; // 第2リンクの長さ（mm）

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
// 逆運動学関数（左アーム）
// ----------------------------
bool inverseKinematicsLeft(float x_p, float y_p, float theta_current, float &theta_sol) {
  float D = (x_p * x_p + y_p * y_p - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (abs(D) > 1.0) {
    return false; // 解なし
  }
  // theta2の可能な解
  float theta2_options[2];
  theta2_options[0] = atan2(sqrt(1 - D * D), D);
  theta2_options[1] = atan2(-sqrt(1 - D * D), D);

  float best_theta1 = 0.0;
  float min_diff = 1e6;

  for (int i = 0; i < 2; i++) {
    float theta2 = theta2_options[i];
    float theta1 = atan2(y_p, x_p) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    theta1 = degrees(theta1);
    theta2 = degrees(theta2);

    // 現在の角度との差分を計算
    float diff = abs(angleDifference(theta1, theta_current));

    if (diff < min_diff) {
      min_diff = diff;
      theta_sol = theta1;
    }
  }

  return true; // 解が見つかった
}

// ----------------------------
// 逆運動学関数（右アーム）
// ----------------------------
bool inverseKinematicsRight(float x_p, float y_p, float theta_current, float &theta_sol) {
  // 右モーターの座標系に変換
  float x = x_p - l0;
  float y = y_p;

  float D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (abs(D) > 1.0) {
    return false; // 解なし
  }
  // theta2の可能な解
  float theta2_options[2];
  theta2_options[0] = atan2(sqrt(1 - D * D), D);
  theta2_options[1] = atan2(-sqrt(1 - D * D), D);

  float best_theta1 = 0.0;
  float min_diff = 1e6;

  for (int i = 0; i < 2; i++) {
    float theta2 = theta2_options[i];
    float theta1 = atan2(y, x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    theta1 = degrees(theta1);
    theta2 = degrees(theta2);

    // 現在の角度との差分を計算
    float diff = abs(angleDifference(theta1, theta_current));

    if (diff < min_diff) {
      min_diff = diff;
      theta_sol = theta1;
    }
  }

  return true; // 解が見つかった
}

// ----------------------------
// PID制御関数
// ----------------------------
float pidControl(float setpoint, float measured, float &integral, float &prevError, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
  return output;
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
// セットアップ関数
// ----------------------------

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
  prevTime = millis();
}

// ----------------------------
// ループ関数
// ----------------------------

void loop() {
  // 現在時間を取得
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= controlInterval) {
    float dt = (currentTime - prevTime) / 1000.0; // 秒に変換

    // 目標位置を設定（ここでは固定値だが、動的に変更可能）
    // 例: x_target = ...; y_target = ...;

    // 逆運動学の計算（左アーム）
    bool leftIK = inverseKinematicsLeft(x_target, y_target, theta_current_left, theta_target_left);

    // 逆運動学の計算（右アーム）
    bool rightIK = inverseKinematicsRight(x_target, y_target, theta_current_right, theta_target_right);

    if (!leftIK || !rightIK) {
      Serial.println("Inverse Kinematics solution not found for the given position.");
      // エラー処理：モータを停止するなど
      setMotorRight(0);
      setMotorLeft(0);
    } else {
      // エンコーダから現在の角度を計算
      theta_current_right = (rightEncoderCount / (float)CPR) * 360.0 / GearRatio;
      theta_current_left = (leftEncoderCount / (float)CPR) * 360.0 / GearRatio;

      // PID制御（右アーム）
      float controlSignal_right = pidControl(theta_target_right, theta_current_right, integral_right, prevError_right, dt);

      // PID制御（左アーム）
      float controlSignal_left = pidControl(theta_target_left, theta_current_left, integral_left, prevError_left, dt);

      // 制御信号をモータ速度に変換（-255 ~ 255）
      float motorSpeed_right = constrain(controlSignal_right, -255, 255);
      float motorSpeed_left = constrain(controlSignal_left, -255, 255);

      // モータに制御信号を送信
      setMotorRight(motorSpeed_right);
      setMotorLeft(motorSpeed_left);

      // デバッグ用シリアル出力
      Serial.print("Theta Target Right: "); Serial.print(theta_target_right);
      Serial.print(" | Theta Current Right: "); Serial.print(theta_current_right);
      Serial.print(" | Control Right: "); Serial.print(motorSpeed_right);
      Serial.print(" || Theta Target Left: "); Serial.print(theta_target_left);
      Serial.print(" | Theta Current Left: "); Serial.print(theta_current_left);
      Serial.print(" | Control Left: "); Serial.println(motorSpeed_left);
    }

    // 時間を更新
    prevTime = currentTime;
  }
}
