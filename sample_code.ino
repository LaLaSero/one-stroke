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

// 制御周期
unsigned long prevTime = 0;
const unsigned long controlInterval = 100; // ミリ秒

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
  if (speed > 0) {
    digitalWrite(right_IN1, HIGH);
    analogWrite(right_IN2, map(speed, 0, 255, 0, 255));
  } else if (speed < 0) {
    digitalWrite(right_IN2, HIGH);
    analogWrite(right_IN1, map(-speed, 0, 255, 0, 255));
  } else {
    digitalWrite(right_IN1, LOW);
    digitalWrite(right_IN2, LOW);
  }
}

void setMotorLeft(float speed) {
  // speed: -255 ~ 255
  if (speed > 0) {
    digitalWrite(left_IN1, HIGH);
    analogWrite(left_IN2, map(speed, 0, 255, 0, 255));
  } else if (speed < 0) {
    digitalWrite(left_IN2, HIGH);
    analogWrite(left_IN1, map(-speed, 0, 255, 0, 255));
  } else {
    digitalWrite(left_IN1, LOW);
    digitalWrite(left_IN2, LOW);
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

  // モータを停止
  setMotorRight(0);
  setMotorLeft(0);

  Serial.println("start");//動作検証用サンプルコード開始
}

// ----------------------------
// ループ関数
// ----------------------------

void loop() {
  // 現在時間を取得
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= controlInterval) {
    // 前回の時間との差を計算
    float dt = (currentTime - prevTime) / 1000.0; // 秒に変換

    // テストパターン:
    // 1. 右側モータを正方向に回転（例えば、90度）
    // 2. 一定時間回転させる
    // 3. 右側モータを停止
    // 4. 左側モータを反方向に回転（例えば、-90度）
    // 5. 一定時間回転させる
    // 6. 左側モータを停止

    // ここでは単純な前後回転を行います

    // 右側モータを正方向に回転
    Serial.println("right_plus");//右側モータを正方向に回転開始
    setMotorRight(150); // PWM値を150で回転
    delay(2000); // 2秒間回転
    setMotorRight(0); // モータ停止
    Serial.println("stop");//右側モータ停止

    // 左側モータを反方向に回転
    Serial.println("left_minus");//左側モータを反方向に回転開始
    setMotorLeft(-150); // PWM値を-150で回転
    delay(2000); // 2秒間回転
    setMotorLeft(0); // モータ停止
    Serial.println("stop");//左側モータ停止

    // エンコーダーカウントから角度を計算
    float theta1_current = (rightEncoderCount / (float)CPR) * 360.0 / GearRatio;
    float theta2_current = (leftEncoderCount / (float)CPR) * 360.0 / GearRatio;

    // デバッグ用シリアル出力
    Serial.print("rightEncoderCount: "); Serial.print(rightEncoderCount);//右側エンコーダーカウント
    Serial.print(" |right_encoder: "); Serial.print(theta1_current);//| 右側角度
    Serial.print(" ||leftEncoderCount: "); Serial.print(leftEncoderCount);//|| 左側エンコーダーカウント
    Serial.print(" | left_encoder: "); Serial.println(theta2_current);//左側角度

    // 次の制御周期まで待機
    prevTime = currentTime;
  }
}