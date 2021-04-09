#pragma once

#include <cmath>

void Solve2BarFK(float &xp, float &yp, float la, float lb, float theta1,
                 float theta2) {
  xp = la * cosf(theta1) + lb * cosf(theta1 + theta2);
  yp = la * sinf(theta1) + lb * sinf(theta1 + theta2);
}

bool Solve2BarIK(float xp, float yp, float la, float lb, float &theta1,
                 float &theta2, bool up_elbow) {
  float cos2 = (xp * xp + yp * yp - la * la - lb * lb) / (2.f * la * lb);
  float sin2 = sqrtf(1 - cos2 * cos2);
  if (up_elbow) {
    sin2 = -sin2;
  }
  theta2 = atan2(sin2, cos2);

  float k1 = la + lb * cos2;
  float k2 = lb * sin2;

  float sin1 = (yp * k1 - xp * k2) / (k1 * k1 + k2 * k2);
  float cos1 = (yp - k1 * sin1) / k2;

  theta1 = atan2(sin1, cos1);

  return true;
}

bool Solve5BarWithShift(float xp, float yp, float la, float lb1, float lb2,
                        float b1_b2_angle, float lc, float &theta1,
                        float &theta4, Direction dir) {
  float theta2;
  float theta3;
  float cx;
  float cy;

  // if (!Solve2BarIK(xp, yp, la, lb2, theta1, theta2, 0)) {
  //  return false;
  //}

  // Solve2BarFK(cx, cy, la, lb1, theta1, theta2 - b1_b2_angle);
  // if (!Solve2BarIK(cx - lc, cy, la, lb1, theta4, theta3, 1)) {
  //  return false;
  //}

  switch (dir) {
    case Direction::LEFT:
      Solve2BarIK(xp, yp, la, lb2, theta1, theta2, false);
      Solve2BarFK(cx, cy, la, lb1, theta1, theta2 + b1_b2_angle);
      Solve2BarIK(cx - lc, cy, la, lb1, theta4, theta3, true);
      break;
    case Direction::RIGHT:
      Solve2BarIK(xp - lc, yp, la, lb2, theta4, theta3, true);
      Solve2BarFK(cx, cy, la, lb1, theta4, theta3 - b1_b2_angle);
      Solve2BarIK(cx + lc, cy, la, lb1, theta1, theta2, false);
      break;
  }

  // Serial.printf("theta1: %f, theta4: %f, xp: %f, yp: %f\r\n",
  //                theta1 - M_PI_2, theta4 - M_PI_2, xp, yp);
}

bool Solve90DegIK(float xp, float yp, float la, float &alpha, float &length) {
  float l = sqrtf(xp * xp + yp * yp - la * la);
  alpha = atan2f(l * xp - la * yp, la * xp + l * yp);
  length = -l;
  return true;
}
