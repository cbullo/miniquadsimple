#include "control.h"

#include "Arduino.h"
#include "config.h"

#undef abs
#include "Eigen.h"
#include "Eigen/Geometry"

void Robot::ApplyConfig() {
  for (int l = 0; l < 4; ++l) {
    for (int s = 0; s < 3; ++s) {
      for (int p = 0; p < 2; ++p) {
        legs[l]->GetServo(s)->SetCalibrationPoint(
            p, config.servo_calibration[l][s][p].us,
            config.servo_calibration[l][s][p].angle);
      }
    }
  }
}

void Robot::Init(LegConfig* FL_config, LegConfig* FR_config,
                 LegConfig* BL_config, LegConfig* BR_config) {
  ApplyConfig();

  FL.Init(FL_config, 2, 9, 3);
  FR.Init(FR_config, 0, 1, 5);
  BL.Init(BL_config, 15, 14, 8);
  BR.Init(BR_config, 22, 23, 4);

  FL.Attach();
  FR.Attach();
  BL.Attach();
  BR.Attach();
}

void Robot::Disable() {
  for (int i = 0; i < 4; ++i) {
    GetLeg(0)->Detach();
  }
}

ControlBase::ControlBase(Robot* robot) : actor_(robot) {}

ControlAngles::ControlAngles(Robot* robot) : ControlBase(robot) {}

void ControlAngles::ProcessInput(float axes[6], uint32_t buttons) {
  float z_offset = axes[1];

  // Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", axes[0], axes[1],
  //               axes[2], axes[5]);

  z_offset = z_offset;
  Serial.printf("%f\r\n", z_offset);

  GetActor()->GetLegFL()->GetServo(0)->SetPosition(0.75f * z_offset * -M_PI /
                                                   2);
  GetActor()->GetLegFL()->GetServo(1)->SetPosition(0.75f * z_offset * M_PI / 2);
  GetActor()->GetLegFL()->GetServo(2)->SetPosition(0.f);

  GetActor()->GetLegFR()->GetServo(0)->SetPosition(0.75f * z_offset * -M_PI /
                                                   2);
  GetActor()->GetLegFR()->GetServo(1)->SetPosition(0.75f * z_offset * M_PI / 2);
  GetActor()->GetLegFR()->GetServo(2)->SetPosition(0.f);

  GetActor()->GetLegBL()->GetServo(0)->SetPosition(0.75f * z_offset * -M_PI /
                                                   2);
  GetActor()->GetLegBL()->GetServo(1)->SetPosition(0.75f * z_offset * M_PI / 2);
  GetActor()->GetLegBL()->GetServo(2)->SetPosition(0.f);

  GetActor()->GetLegBR()->GetServo(0)->SetPosition(0.75f * z_offset * -M_PI /
                                                   2);
  GetActor()->GetLegBR()->GetServo(1)->SetPosition(0.75f * z_offset * M_PI / 2);
  GetActor()->GetLegBR()->GetServo(2)->SetPosition(0.f);
}

Control2DIK::Control2DIK(Robot* robot) : ControlBase(robot) {}

void Control2DIK::ProcessInput(float axes[6], uint32_t buttons) {
  float x_offset = 20.f * (axes[0] - 0.5f);
  // float xr_offset = 6.5 + 50.f * (axes[0] - 0.5f);
  float z_offset = -40.f - 20.f * axes[1];
  // float x_offset = 6.5f;
  // float y_offset = 55.f;

  // Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", axes[0], axes[1],
  //               axes[2], axes[5]);

  // float theta1l = 0.f;
  // float theta4l = 0.f;
  float theta1r = 0.f;
  float theta4r = 0.f;

  auto* FR = GetActor()->GetLegFR();
  FR->Solve2DLeg(x_offset, z_offset, theta1r, theta4r);
  FR->GetServo(0)->SetPosition(theta1r);
  FR->GetServo(1)->SetPosition(theta4r);
  FR->GetServo(2)->SetPosition(0.f);

  // auto* FL = GetActor()->GetLegFL();
  // FL->GetServo(0)->SetPosition(theta4l);
  // FL->GetServo(1)->SetPosition(theta1l);
  // FL->GetServo(2)->SetPosition(0.f);

  // BL->Solve2DLeg(xr_offset, y_offset, theta1r, theta4r);
  // auto* BR = GetActor()->GetLegBR();
  // BR->GetServo(0)->SetPosition(theta4r);
  // BR->GetServo(1)->SetPosition(theta1r);
  // BR->GetServo(2)->SetPosition(0.f);

  // auto* FR = GetActor()->GetLegFR();
  // FR->GetServo(0)->SetPosition(theta4r);
  // FR->GetServo(1)->SetPosition(theta1r);
  // FR->GetServo(2)->SetPosition(0.f);

  // Serial.printf("theta1l: %f, theta4l: %f, xl_offset: %f, z_offset: %f\r\n",
  //              theta1r, theta4r, x_offset, z_offset);
}

ControlIK::ControlIK(Robot* actor) : ControlBase(actor){};
void ControlIK::ProcessInput(float axes[6], uint32_t buttons) {
  float x_offset = 0.f;    // * (0.5f - axes[1]);
  float y_offset = 0.f;    // * (axes[0] - 0.5f);
  float z_offset = -60.f;  // + 20.f * (axes[5] - 0.5f);

  Eigen::Vector3f offset(x_offset, y_offset, z_offset);

  Eigen::Affine3f transformation;
  transformation = Eigen::AngleAxisf(-0.15f * (0.5f - axes[1]) * M_PI,
                                     Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(-0.15f * (0.5f - axes[2]) * M_PI,
                                     Eigen::Vector3f::UnitZ()) *
                   Eigen::AngleAxisf(0.15f * (0.5f - axes[0]) * M_PI,
                                     Eigen::Vector3f::UnitX());

  auto* FL = GetActor()->GetLegFL();
  auto* FR = GetActor()->GetLegFR();
  auto* BL = GetActor()->GetLegBL();
  auto* BR = GetActor()->GetLegBR();

  Eigen::Vector3f FL_target =
      FL->GetConfig().offset + offset + Eigen::Vector3f(13.f, 22.f, 0.f);

  Eigen::Vector3f FR_target =
      FR->GetConfig().offset + offset + Eigen::Vector3f(13.f, -22.f, 0.f);

  Eigen::Vector3f BL_target =
      BL->GetConfig().offset + offset + Eigen::Vector3f(0.f, 22.f, 0.f);

  Eigen::Vector3f BR_target =
      BR->GetConfig().offset + offset + Eigen::Vector3f(0.f, -22.f, 0.f);

  FL_target = transformation * FL_target;
  FR_target = transformation * FR_target;
  BL_target = transformation * BL_target;
  BR_target = transformation * BR_target;

  FL->SetEffectorTarget(FL_target);
  FR->SetEffectorTarget(FR_target);
  BL->SetEffectorTarget(BL_target);
  BR->SetEffectorTarget(BR_target);
};
