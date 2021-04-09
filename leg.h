#pragma once

#include "PWMServo.h"
#include "calibration.h"
#include "types.h"

class Motor {
 public:
  void Attach();
  void Detach();
  void Init(int pin);
  void SetPosition(float angle);
  // void SetPositionUs(int us_angle);
  void SetPositionDeg(int deg);
  void SetCalibrationPoint(int index, int us, float angle);

  int servo_pin_;

 private:
  PWMServo servo_;

  CalibrationPoint calibration_points_[2];
};

struct LegConfig {
  float la;
  float lb1;
  float lb2;
  float b1_b2_angle;
  float lc;
  float ld;
};

class Leg {
 public:
  void Attach();
  void Init(LegConfig* config, int front, int back, int side);
  // void SetPosition(float front_angle, float back_angle, float side_angle);
  Motor* GetServo(int index) {
    switch (index) {
      case 0:
        return &front_;
      case 1:
        return &back_;
      case 2:
        return &side_;
    }

    return &front_;
  }

  bool Solve2DLeg(float xp, float yp, float &theta1, float &theta4, Direction dir);
  bool SolveSideLeg(float xp, float yp, float &alpha, float &length);

 private:
  Motor front_;
  Motor back_;
  Motor side_;

  LegConfig* config_;
};