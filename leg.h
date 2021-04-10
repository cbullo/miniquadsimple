#pragma once

#include "Eigen.h"
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

struct LegDimensions {
  float la;
  float lb1;
  float lb2;
  float b1_b2_angle;
  float lc;
  float ld;
};

struct LegConfig {
  Eigen::Vector3f offset;
  float swing_direction;
  LegDimensions *dimensions;
};

class Leg {
 public:
  void Attach();
  void Init(LegConfig *config, int front, int back, int side);
  // void SetPosition(float front_angle, float back_angle, float side_angle);
  Motor *GetServo(int index) {
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

  bool Solve2DLeg(float xp, float zp, float &theta1, float &theta4);
  bool SolveSideLeg(float yp, float zp, float &alpha, float &length);

  bool SetEffectorTarget(const Eigen::Vector3f& target);
  const LegConfig &GetConfig() const { return *config_; }

 private:
  Motor front_;
  Motor back_;
  Motor side_;

  LegConfig *config_;
};