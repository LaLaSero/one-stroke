#ifndef REVERSE_KINEMATICS_H
#define REVERSE_KINEMATICS_H 

#include "config.h"

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

  return true;
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

  return true;
}

#endif