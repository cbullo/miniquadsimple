#include "control.h"

#include "Arduino.h"
#include "config.h"

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

void Robot::Init(LegConfig* leg_config) {
  ApplyConfig();

  FL.Init(leg_config, 2, 9, 3);
  FR.Init(leg_config, 0, 1, 5);
  BL.Init(leg_config, 15, 14, 8);
  BR.Init(leg_config, 22, 23, 4);

  FL.Attach();
  FR.Attach();
  BL.Attach();
  BR.Attach();
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
  FR->Solve2DLeg(x_offset, z_offset, theta1r, theta4r, Direction::RIGHT);
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

Control2DSideIK::Control2DSideIK(Robot* actor) : ControlBase(actor){};
void Control2DSideIK::ProcessInput(float axes[6], uint32_t buttons) {
  float x_offset = 40.f * (0.5f - axes[1]);
  float y_offset = 30.f * (0.5f - axes[0]);
  float z_offset = -50.f + 20.f * (axes[5] - 0.5f);

  float alpha = 0.f;
  float length = 40.f;
  float theta1 = 0.f;
  float theta4 = 0.f;

  auto* FL = GetActor()->GetLegFL();
  FL->SolveSideLeg(22.f + y_offset, z_offset, alpha, length);
  FL->Solve2DLeg(-x_offset, length, theta1, theta4, Direction::LEFT);

  FL->GetServo(0)->SetPosition(theta1);
  FL->GetServo(1)->SetPosition(theta4);
  FL->GetServo(2)->SetPosition(alpha);

  auto* FR = GetActor()->GetLegFR();
  FR->SolveSideLeg(22.f - y_offset, z_offset, alpha, length);
  FR->Solve2DLeg(x_offset + 13.f, length, theta1, theta4, Direction::RIGHT);

  FR->GetServo(0)->SetPosition(theta1);
  FR->GetServo(1)->SetPosition(theta4);
  FR->GetServo(2)->SetPosition(alpha);

  auto* BL = GetActor()->GetLegBL();
  BL->SolveSideLeg(22.f + y_offset, z_offset, alpha, length);
  BL->Solve2DLeg(-x_offset + 13, length, theta1, theta4, Direction::LEFT);

  BL->GetServo(0)->SetPosition(theta1);
  BL->GetServo(1)->SetPosition(theta4);
  BL->GetServo(2)->SetPosition(alpha);

  auto* BR = GetActor()->GetLegBR();
  BR->SolveSideLeg(22.f - y_offset, z_offset, alpha, length);
  BR->Solve2DLeg(x_offset, length, theta1, theta4, Direction::RIGHT);

  BR->GetServo(0)->SetPosition(theta1);
  BR->GetServo(1)->SetPosition(theta4);
  BR->GetServo(2)->SetPosition(alpha);

  Serial.printf(
      "alpha: %f, theta_1: %f, theta_4: %f, length: %f, y_offset: %f, "
      "z_offset: %f\r\n",
      alpha, theta1, theta4, length, y_offset, z_offset);
};