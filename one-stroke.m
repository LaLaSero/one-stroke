#include <Servo.h>

// リンクの長さ（全て同じ長さとする）
const float L = 100.0; // 単位：mm

// モーターのピン番号
const int motorPin1 = 9;
const int motorPin2 = 10;

// サーボモータのオブジェクト
Servo motor1;
Servo motor2;

// PID制御用のパラメータ
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// PID制御用の変数
float integral1 = 0;
float integral2 = 0;
float prevError1 = 0;
float prevError2 = 0;
unsigned long prevTime = 0;

// 目標位置（先端のペンの座標）
float x_target = 150.0; // 単位：mm
float y_target = 50.0;  // 単位：mm

// 現在の駆動関節角度（初期値）
float theta1 = 0.0;
float theta2 = 0.0;

// モーターの目標角度
float theta1_target = 0.0;
float theta2_target = 0.0;

// 関数宣言
void inverseKinematics(float x, float y, float& theta1, float& theta2);
float pidControl(float setpoint, float measured, float& integral, float& prevError, float dt);

void setup() {
  // サーボモータの初期化
  motor1.attach(motorPin1);
  motor2.attach(motorPin2);

  // 初期時間を取得
  prevTime = millis();
}

void loop() {
  // 時間の計算
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // 秒に変換

  // 目標位置を設定（ここでは固定値だが、動的に変更可能）
  // x_target = ...;
  // y_target = ...;

  // 逆運動学の計算
  inverseKinematics(x_target, y_target, theta1_target, theta2_target);

  // モーターの現在の角度を取得（センサーがない場合は前回の出力を仮定）
  float theta1_current = theta1;
  float theta2_current = theta2;

  // PID制御
  float controlSignal1 = pidControl(theta1_target, theta1_current, integral1, prevError1, dt);
  float controlSignal2 = pidControl(theta2_target, theta2_current, integral2, prevError2, dt);

  // 制御信号を角度に変換（サーボモータの範囲は0〜180度）
  float servoAngle1 = constrain(controlSignal1, 0, 180);
  float servoAngle2 = constrain(controlSignal2, 0, 180);

  // モーターに出力
  motor1.write(servoAngle1);
  motor2.write(servoAngle2);

  // 現在の角度を更新
  theta1 = servoAngle1;
  theta2 = servoAngle2;

  // 時間とエラーを更新
  prevTime = currentTime;

  // 制御周期の待機
  delay(10); // 10msごとに制御
}

// 逆運動学を数値的に解く関数
void inverseKinematics(float x, float y, float& theta1_sol, float& theta2_sol) {
  // 初期推定値（前回の解を利用）
  float theta1 = theta1_sol;
  float theta2 = theta2_sol;

  // 収束条件
  const int maxIterations = 100;
  const float tolerance = 0.01; // 許容誤差

  for (int i = 0; i < maxIterations; i++) {
    // フォワードキネマティクスで現在の先端位置を計算
    float x_current = L * cos(radians(theta1)) + L * cos(radians(theta1 + theta2));
    float y_current = L * sin(radians(theta1)) + L * sin(radians(theta1 + theta2));

    // 誤差を計算
    float dx = x - x_current;
    float dy = y - y_current;

    // ヤコビ行列の計算
    float J11 = -L * sin(radians(theta1)) - L * sin(radians(theta1 + theta2));
    float J12 = -L * sin(radians(theta1 + theta2));
    float J21 = L * cos(radians(theta1)) + L * cos(radians(theta1 + theta2));
    float J22 = L * cos(radians(theta1 + theta2));

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

    // 角度を更新
    theta1 += degrees(dtheta1);
    theta2 += degrees(dtheta2);

    // 収束判定
    if (sqrt(dx * dx + dy * dy) < tolerance) {
      break;
    }
  }

  // 解を返す
  theta1_sol = theta1;
  theta2_sol = theta2;
}

// PID制御関数
float pidControl(float setpoint, float measured, float& integral, float& prevError, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
  return output;
}




右側 IN1 ・ ・ ・ Arduino 6
◼右側 IN2 ・ ・ ・ Arduino 7
◼右側 OUT1 ・ ・ ・ Arduino 18
◼右側 OUT2 ・ ・ ・ Arduino 19
◼左側 IN1 ・ ・ ・ Arduino 12
◼左側 IN2 ・ ・ ・ Arduino 13
◼左側 OUT1 ・ ・ ・ Arduino 20
◼左側 OUT2 ・ ・ ・ Arduino 21
左右のモータに対する入出力がこのように指定されている時，
#include <Servo.h>

// リンクの長さ（全て同じ長さとする）
const float L = 100.0; // 単位：mm

// モーターのピン番号
const int motorPin1 = 9;
const int motorPin2 = 10;

// サーボモータのオブジェクト
Servo motor1;
Servo motor2;

// PID制御用のパラメータ
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

// PID制御用の変数
float integral1 = 0;
float integral2 = 0;
float prevError1 = 0;
float prevError2 = 0;
unsigned long prevTime = 0;

// 目標位置（先端のペンの座標）
float x_target = 150.0; // 単位：mm
float y_target = 50.0;  // 単位：mm

// 現在の駆動関節角度（初期値）
float theta1 = 0.0;
float theta2 = 0.0;

// モーターの目標角度
float theta1_target = 0.0;
float theta2_target = 0.0;

// 関数宣言
void inverseKinematics(float x, float y, float& theta1, float& theta2);
float pidControl(float setpoint, float measured, float& integral, float& prevError, float dt);

void setup() {
  // サーボモータの初期化
  motor1.attach(motorPin1);
  motor2.attach(motorPin2);

  // 初期時間を取得
  prevTime = millis();
}

void loop() {
  // 時間の計算
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // 秒に変換

  // 目標位置を設定（ここでは固定値だが、動的に変更可能）
  // x_target = ...;
  // y_target = ...;

  // 逆運動学の計算
  inverseKinematics(x_target, y_target, theta1_target, theta2_target);

  // モーターの現在の角度を取得（センサーがない場合は前回の出力を仮定）
  float theta1_current = theta1;
  float theta2_current = theta2;

  // PID制御
  float controlSignal1 = pidControl(theta1_target, theta1_current, integral1, prevError1, dt);
  float controlSignal2 = pidControl(theta2_target, theta2_current, integral2, prevError2, dt);

  // 制御信号を角度に変換（サーボモータの範囲は0〜180度）
  float servoAngle1 = constrain(controlSignal1, 0, 180);
  float servoAngle2 = constrain(controlSignal2, 0, 180);

  // モーターに出力
  motor1.write(servoAngle1);
  motor2.write(servoAngle2);

  // 現在の角度を更新
  theta1 = servoAngle1;
  theta2 = servoAngle2;

  // 時間とエラーを更新
  prevTime = currentTime;

  // 制御周期の待機
  delay(10); // 10msごとに制御
}

// 逆運動学を数値的に解く関数
void inverseKinematics(float x, float y, float& theta1_sol, float& theta2_sol) {
  // 初期推定値（前回の解を利用）
  float theta1 = theta1_sol;
  float theta2 = theta2_sol;

  // 収束条件
  const int maxIterations = 100;
  const float tolerance = 0.01; // 許容誤差

  for (int i = 0; i < maxIterations; i++) {
    // フォワードキネマティクスで現在の先端位置を計算
    float x_current = L * cos(radians(theta1)) + L * cos(radians(theta1 + theta2));
    float y_current = L * sin(radians(theta1)) + L * sin(radians(theta1 + theta2));

    // 誤差を計算
    float dx = x - x_current;
    float dy = y - y_current;

    // ヤコビ行列の計算
    float J11 = -L * sin(radians(theta1)) - L * sin(radians(theta1 + theta2));
    float J12 = -L * sin(radians(theta1 + theta2));
    float J21 = L * cos(radians(theta1)) + L * cos(radians(theta1 + theta2));
    float J22 = L * cos(radians(theta1 + theta2));

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

    // 角度を更新
    theta1 += degrees(dtheta1);
    theta2 += degrees(dtheta2);

    // 収束判定
    if (sqrt(dx * dx + dy * dy) < tolerance) {
      break;
    }
  }

  // 解を返す
  theta1_sol = theta1;
  theta2_sol = theta2;
}

// PID制御関数
float pidControl(float setpoint, float measured, float& integral, float& prevError, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;
  return output;
}
