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
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// PID制御用の変数（右側）
float integral1 = 0;
float prevError1 = 0;

// PID制御用の変数（左側）
float integral2 = 0;
float prevError2 = 0;

// 制御周期
unsigned long prevTime = 0;
const unsigned long controlInterval = 10; // ミリ秒

// 目標位置（先端のペンの座標）
float x_target = 150.0; // 単位：mm
float y_target = 50.0;  // 単位：mm

// 現在の駆動関節角度（degrees）
float theta1 = 0.0;
float theta2 = 0.0;

// モーターの目標角度（degrees）
float theta1_target = 0.0;
float theta2_target = 0.0;

// リンクの長さ（全て同じ長さとする）
const float L = 100.0; // 単位：mm

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
// 逆運動学関数
// ----------------------------

void inverseKinematics(float x, float y, float &theta1_sol, float &theta2_sol) {
  // ニュートン–ラフソン法を用いた逆運動学の数値解法
  const int maxIterations = 100;
  const float tolerance = 0.01; // 許容誤差（mm）

  float theta1_local = theta1_sol;
  float theta2_local = theta2_sol;

  for (int i = 0; i < maxIterations; i++) {
    // フォワードキネマティクスで現在の先端位置を計算
    float x_current = L * cos(radians(theta1_local)) + L * cos(radians(theta1_local + theta2_local));
    float y_current = L * sin(radians(theta1_local)) + L * sin(radians(theta1_local + theta2_local));

    // 誤差を計算
    float dx = x - x_current;
    float dy = y - y_current;
    float error = sqrt(dx * dx + dy * dy);

    // 収束判定
    if (error < tolerance) {
      break;
    }

    // ヤコビ行列の計算
    float J11 = -L * sin(radians(theta1_local)) - L * sin(radians(theta1_local + theta2_local));
    float J12 = -L * sin(radians(theta1_local + theta2_local));
    float J21 = L * cos(radians(theta1_local)) + L * cos(radians(theta1_local + theta2_local));
    float J22 = L * cos(radians(theta1_local + theta2_local));

    // ヤコビ行列の逆行列を計算
    float det = J11 * J22 - J12 * J21;
    if (abs(det) < 1e-6) {
      // 行列が特異な場合
      break;
    }
    float invJ11 = J22 / det;
    float invJ12 = -J12 / det;
    float invJ21 = -J21 / det;
    float invJ22 = J11 / det;

    // 角度の更新量を計算
    float dtheta1 = invJ11 * dx + invJ12 * dy;
    float dtheta2 = invJ21 * dx + invJ22 * dy;

    // 角度を更新（ラジアンから度に変換）
    theta1_local += degrees(dtheta1);
    theta2_local += degrees(dtheta2);
  }

  // 解を返す
  theta1_sol = theta1_local;
  theta2_sol = theta2_local;
}

// ----------------------------
// モータ制御関数
// ----------------------------

void setMotorRight(float speed) {
  // speed: -255 ~ 255
  if (speed > 0) {
    digitalWrite(right_IN1, HIGH);
    analogWrite(right_IN2, map(speed, 0, 255, 0, 255));
  } else {
    digitalWrite(right_IN2, HIGH);
    analogWrite(right_IN1, map(-speed, 0, 255, 0, 255));
  }
}

void setMotorLeft(float speed) {
  // speed: -255 ~ 255
  if (speed > 0) {
    digitalWrite(left_IN1, HIGH);
    analogWrite(left_IN2, map(speed, 0, 255, 0, 255));
  } else {
    digitalWrite(left_IN2, HIGH);
    analogWrite(left_IN1, map(-speed, 0, 255, 0, 255));
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

    // 逆運動学の計算
    inverseKinematics(x_target, y_target, theta1_target, theta2_target);

    // エンコーダから現在の角度を計算
    // エンコーダカウントから角度を計算
    float theta1_current = (rightEncoderCount / (float)CPR) * 360.0 / GearRatio;
    float theta2_current = (leftEncoderCount / (float)CPR) * 360.0 / GearRatio;

    // PID制御
    float controlSignal1 = pidControl(theta1_target, theta1_current, integral1, prevError1, dt);
    float controlSignal2 = pidControl(theta2_target, theta2_current, integral2, prevError2, dt);

    // 制御信号をモータ速度に変換（-255 ~ 255）
    float motorSpeed1 = constrain(controlSignal1, -255, 255);
    float motorSpeed2 = constrain(controlSignal2, -255, 255);

    // モータに制御信号を送信
    setMotorRight(motorSpeed1);
    setMotorLeft(motorSpeed2);

    // デバッグ用シリアル出力
    Serial.print("Theta1 Target: "); Serial.print(theta1_target);
    Serial.print(" | Theta1 Current: "); Serial.print(theta1_current);
    Serial.print(" | Control1: "); Serial.print(motorSpeed1);
    Serial.print(" || Theta2 Target: "); Serial.print(theta2_target);
    Serial.print(" | Theta2 Current: "); Serial.print(theta2_current);
    Serial.print(" | Control2: "); Serial.println(motorSpeed2);

    // 時間とエラーを更新
    prevTime = currentTime;
  }
}
