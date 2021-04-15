#include "leg.h"

#include "Arduino.h"
#include "ik_solver.h"

void Motor::Init(int pin, float min_angle, float max_angle) {
  servo_pin_ = pin;
  min_angle_ = min_angle;
  max_angle_ = max_angle;
}

bool Motor::Attach() {
  if (servo_pin_ < 0 || servo_pin_ >= NUM_DIGITAL_PINS) return false;
  if (!digitalPinHasPWM(servo_pin_)) return false;
  analogWriteFrequency(servo_pin_, 50);
  digitalWrite(servo_pin_, LOW);
  pinMode(servo_pin_, OUTPUT);
  return true;
}

void Motor::Detach() {
  pinMode(servo_pin_, INPUT);
}

void Motor::SetServoPosition(float deg) {

  if (servo_pin_ >= NUM_DIGITAL_PINS) return;
  float usec =
      (float)((max_us_ - min_us_)) * ((float)deg / 180.0f) + (float)(min_us_);
  uint32_t duty = (int)(usec / 20000.0f * 4096.0f);
  //Serial.printf("angle=%.2f, usec=%.2f, duty=%d, min=%d, max=%d\n", deg, usec,
  //              duty, min_us_, max_us_);

  noInterrupts();
  uint32_t oldres = analogWriteResolution(12);
  analogWrite(servo_pin_, duty);
  analogWriteResolution(oldres);
  interrupts();
}

bool Motor::IsInRange(float angle) {
  return min_angle_ <= angle && angle <= max_angle_;
}

// void Motor::SetPositionDeg(float deg) {
//   if (deg < 0 || deg > 180) {
//     Serial.println("Outside range!");
//     return;
//   }

//   servo_.write(deg);
// }

bool Motor::SetPosition(float angle) {
  if (!IsInRange(angle)) {
    return false;
  }

  const float ang_diff =
      calibration_points_[1].angle - calibration_points_[0].angle;
  const float us_diff = calibration_points_[1].us - calibration_points_[0].us;

  const float slope = us_diff / ang_diff;
  const float deg_angle = calibration_points_[0].us +
                          (angle - calibration_points_[0].angle) * slope;

  SetServoPosition(deg_angle);
  return true;
}

void Motor::SetCalibrationPoint(int index, int us, float angle) {
  calibration_points_[index] = {us, angle};
}

void Leg::Attach() {
  front_.Attach();
  back_.Attach();
  side_.Attach();
}

void Leg::Detach() {
  front_.Detach();
  back_.Detach();
  side_.Detach();
}

void Leg::Init(LegConfig *config, int front, int back, int side) {
  config_ = config;
  front_.Init(front, config->dimensions->min_angle[0],
              config->dimensions->max_angle[0]);
  back_.Init(back, config->dimensions->min_angle[1],
             config->dimensions->max_angle[1]);
  side_.Init(side, config->dimensions->min_angle[2],
             config->dimensions->max_angle[2]);
}

bool Leg::Solve2DLeg(float xp, float yp, float &theta1, float &theta4) {
  bool ret = Solve5BarWithShift(
      xp, yp, config_->dimensions->la, config_->dimensions->lb1,
      config_->dimensions->lb2, config_->dimensions->b1_b2_angle,
      config_->dimensions->lc, theta1, theta4, Direction::RIGHT);
  theta1 += M_PI_2;
  theta4 += M_PI_2;
  return ret;
}

bool Leg::SolveSideLeg(float yp, float zp, float &alpha, float &length) {
  yp *= config_->swing_direction;
  bool ret = Solve90DegIK(yp, -zp, config_->dimensions->ld, alpha, length);
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

  // Serial.printf(
  //     "alpha: %f, theta_1: %f, theta_4: %f, length: %f, x_offset: %f, "
  //     "y_offset: %f, "
  //     "z_offset: %f\r\n",
  //     alpha, theta1, theta4, length, local_target(0), local_target(1),
  //     local_target(2));

  return true;
}