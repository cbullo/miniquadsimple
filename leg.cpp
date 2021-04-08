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

bool Leg::Solve2DLeg(float xp, float yp, float &theta1, float &theta4, int dir) {
  bool ret = Solve5BarWithShift(xp, yp, config_->la, config_->lb1, config_->lb2,
                            config_->b1_b2_angle, config_->lc, theta1, theta4, dir);
  theta1 -= M_PI_2;
  theta4 -= M_PI_2;
  return ret;
}

bool Leg::SolveSideLeg(float xp, float yp, float &alpha, float &length) {
  return Solve90DegIK(xp, yp, config_->ld, alpha, length);
}