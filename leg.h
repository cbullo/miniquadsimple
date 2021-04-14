#pragma once

#include "Eigen.h"
#include "calibration.h"
#include "types.h"

class Motor {
 public:
  bool Attach();
  void Detach();
  void Init(int pin, float min_angle, float max_angle);
  bool IsInRange(float angle);
  bool SetPosition(float angle);

  // void SetPositionUs(int us_angle);
  // void SetPositionDeg(int deg);
  void SetCalibrationPoint(int index, int us, float angle);
  void SetServoPosition(float deg);

  int servo_pin_;

 private:
  // PWMServo servo_;
  float min_angle_;
  float max_angle_;

  int min_us_ = 544;
  int max_us_ = 2400;

  CalibrationPoint calibration_points_[2];
  bool disabled_ = false;
};

struct LegDimensions {
  float la;
  float lb1;
  float lb2;
  float b1_b2_angle;
  float lc;
  float ld;
  float min_angle[3];
  float max_angle[3];
};

struct LegConfig {
  Eigen::Vector3f offset;
  float swing_direction;
  LegDimensions *dimensions;
};

class Leg {
 public:
  void Attach();
  void Detach();

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

  bool SetEffectorTarget(const Eigen::Vector3f &target);
  const LegConfig &GetConfig() const { return *config_; }

 private:
  Motor front_;
  Motor back_;
  Motor side_;

  LegConfig *config_;
};