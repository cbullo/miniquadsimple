#include "leg.h"

#include "Arduino.h"
#include "ik_solver.h"

void Motor::Attach() { auto ret = servo_.attach(servo_pin_); }
void Motor::Detach() {
  // servo_.detach();
}

void Motor::Init(int pin) { servo_pin_ = pin; }

// void Motor::SetPositionUs(int us_angle) { servo_.writeMicroseconds(us_angle);
// }
void Motor::SetPositionDeg(int deg) {
  if (deg < 0 || deg > 180) {
    Serial.println("Outside range!");
  }
  servo_.write(deg);
}

void Motor::SetPosition(float angle) {
  const float ang_diff =
      calibration_points_[1].angle - calibration_points_[0].angle;
  const float us_diff = calibration_points_[1].us - calibration_points_[0].us;

  const float slope = us_diff / ang_diff;

  const int us_angle = calibration_points_[0].us +
                       (angle - calibration_points_[0].angle) * slope;

  // Serial.print(calibration_points_[0].us);
  // Serial.print( " ");
  // Serial.print(calibration_points_[0].angle);
  // Serial.print( " ");
  // Serial.print(calibration_points_[1].us);
  // Serial.print( " ");
  // Serial.println(calibration_points_[1].angle);
  // Serial.println(us_angle);
  SetPositionDeg(us_angle);
}

void Motor::SetCalibrationPoint(int index, int us, float angle) {
  calibration_points_[index] = {us, angle};
}

void Leg::Attach() {
  front_.Attach();
  back_.Attach();
  side_.Attach();
}

void Leg::Init(LegConfig *config, int front, int back, int side) {
  config_ = config;
  front_.Init(front);
  back_.Init(back);
  side_.Init(side);
}

bool Leg::Solve2DLeg(float xp, float yp, float &theta1, float &theta4) {
  bool ret = Solve5BarWithShift(
      xp, yp, config_->dimensions->la, config_->dimensions->lb1,
      config_->dimensions->lb2, config_->dimensions->b1_b2_angle,
      config_->dimensions->lc, theta1, theta4, Direction::RIGHT);
  theta1 += M_PI_2;
  theta4 += M_PI_2;

  // while (theta1 < M_PI) {
  //   theta1 += 2.f * M_PI;
  // }

  // while (theta1 > M_PI) {
  //   theta1 -= 2.f * M_PI;
  // }

  // while (theta4 < M_PI) {
  //   theta4 += 2.f * M_PI;
  // }

  // while (theta4 > M_PI) {
  //   theta4 -= 2.f * M_PI;
  // }

  return ret;
}

bool Leg::SolveSideLeg(float yp, float zp, float &alpha, float &length) {
  yp *= config_->swing_direction;
  bool ret = Solve90DegIK(yp, -zp, config_->dimensions->ld, alpha, length);
  // alpha *= config_->swing_direction;
  return ret;
}

bool Leg::SetEffectorTarget(const Eigen::Vector3f &target) {
  float alpha = 0.f;
  float length = 40.f;
  float theta1 = 0.f;
  float theta4 = 0.f;

  Eigen::Vector3f local_target = target - GetConfig().offset;

  SolveSideLeg(local_target(1), local_target(2), alpha, length);
  Solve2DLeg(local_target(0), length + 8.f, theta1, theta4);

  if (config_->swing_direction == -1.f) {
    GetServo(0)->SetPosition(theta1);
    GetServo(1)->SetPosition(theta4);
    GetServo(2)->SetPosition(alpha);
  } else {
    GetServo(0)->SetPosition(-theta4);
    GetServo(1)->SetPosition(-theta1);
    GetServo(2)->SetPosition(alpha);
  }

  Serial.printf(
      "alpha: %f, theta_1: %f, theta_4: %f, length: %f, x_offset: %f, "
      "y_offset: %f, "
      "z_offset: %f\r\n",
      alpha, theta1, theta4, length, local_target(0), local_target(1),
      local_target(2));
}