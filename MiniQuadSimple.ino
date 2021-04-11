#include "config.h"
#include "control.h"
#include "joystick.h"
#include "leg.h"

// Leg FL;
// Leg FR;
// Leg BL;
// Leg BR;

// Leg* legs[4] = {&FL, &FR, &BL, &BR};

const char* names[4] = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
const char* names_servos[4] = {"L", "R", "S"};

LegDimensions leg_dimensions = {
    30.f,
    34.f,
    43.33f,
    (11.3f * M_PI) / 180.f,
    13.f,
    22.f,
    {(-125.f * M_PI) / 180.f, (-40.f * M_PI) / 180.f, (-30.f * M_PI) / 180.f},
    {(40.f * M_PI) / 180.f, (125.f * M_PI) / 180.f, (30.f * M_PI) / 180.f},
};

LegConfig FL_config = {Eigen::Vector3f((95.3f - 13) / 2.f, 42.9f / 2.f, 0.f),
                       1.f, &leg_dimensions};

LegConfig FR_config = {Eigen::Vector3f((95.3f - 13) / 2.f, -42.9f / 2.f, 0.f),
                       -1.f, &leg_dimensions};

LegConfig BL_config = {Eigen::Vector3f(-(95.3f + 13) / 2.f, 42.9f / 2.f, 0.f),
                       1.f, &leg_dimensions};

LegConfig BR_config = {Eigen::Vector3f(-(95.3f + 13) / 2.f, -42.9f / 2.f, 0.f),
                       -1.f, &leg_dimensions};

Motor* calibration_servo_ = nullptr;
Robot robot;

ControlAngles control_angles(&robot);
Control2DIK control_2d_ik(&robot);
ControlIK control_ik(&robot);

ControlBase* control = &control_ik;

void PrintConfig() {
  Serial.println("---------------------------------------------------------");
  for (int l = 0; l < 4; ++l) {
    Serial.println(names[l]);
    for (int s = 0; s < 3; ++s) {
      Serial.print(names_servos[s]);
      for (int p = 0; p < 2; ++p) {
        Serial.print(" (");
        Serial.print(config.servo_calibration[l][s][p].angle);
        Serial.print(", ");
        Serial.print(config.servo_calibration[l][s][p].us);
        Serial.print(")");
      }
      Serial.println();
    }
  }
  Serial.println("---------------------------------------------------------");
}

struct CalibrationData {
  int leg_index = 0;
  int servo_index = 0;
  int point_index = 0;
  float angle;
  int min_us;
  int max_us;
  bool bisecting = false;
} calibration;

void setup() {
  Serial.begin(19200);
  ReadConfig();

  SetupJoystick();

  robot.Init(&FL_config, &FR_config, &BL_config, &BR_config);
}

void FinishCalibration() {
  calibration.bisecting = false;
  calibration_servo_->Detach();

  config
      .servo_calibration[calibration.leg_index][calibration.servo_index]
                        [calibration.point_index]
      .us = (calibration.min_us + calibration.max_us) / 2;
  config
      .servo_calibration[calibration.leg_index][calibration.servo_index]
                        [calibration.point_index]
      .angle = calibration.angle;

  calibration_servo_ = nullptr;
  StoreConfig();
  PrintConfig();
  robot.ApplyConfig();
}

void loop() {
  UpdateJoystick();
  control->ProcessInput(joystick_axes, joystick_buttons);

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil(' ');
    // ctr <leg_index 0..3> <servo_index 0..2> <point_index 0..1>
    if (cmd == "cnf") {
      PrintConfig();
    }
    if (cmd == "ctr") {
      if (Serial.available() > 0) {
        calibration.leg_index = Serial.parseInt();
        calibration.servo_index = Serial.parseInt();
        calibration.point_index = Serial.parseInt();
        if (calibration_servo_) {
          calibration_servo_->Detach();
        }

        calibration_servo_ = robot.GetLeg(calibration.leg_index)
                                 ->GetServo(calibration.servo_index);
        Serial.println(calibration_servo_->servo_pin_);
        calibration_servo_->Attach();
        calibration.bisecting = true;
      }
    } else if (cmd == "set") {
      if (Serial.available() > 0) {
        calibration.angle = (Serial.parseFloat() * M_PI) / 180.0;
        int val = Serial.parseInt();
        calibration.min_us = val;
        calibration.max_us = val;
        calibration_servo_->SetServoPosition(val);
      }
      // bis <deg_angle> <lower_bound_us> <upper_bound_us>
    } else if (cmd == "bis") {
      calibration.angle = (Serial.parseFloat() * M_PI) / 180.0;
      calibration.min_us = Serial.parseInt();
      calibration.max_us = Serial.parseInt();
      calibration_servo_->SetServoPosition(
          (calibration.min_us + calibration.max_us) / 2);
    } else if (calibration.bisecting) {
      if (cmd == "u") {
        calibration.min_us = (calibration.min_us + calibration.max_us) / 2;
        if (calibration.min_us == calibration.max_us) {
          FinishCalibration();
        } else {
          calibration_servo_->SetServoPosition(
              (calibration.min_us + calibration.max_us) / 2);
        }
      } else if (cmd == "d") {
        calibration.max_us = (calibration.min_us + calibration.max_us) / 2;
        if (calibration.min_us == calibration.max_us) {
          FinishCalibration();
        } else {
          calibration_servo_->SetServoPosition(
              (calibration.min_us + calibration.max_us) / 2);
        }
      } else if (cmd == "f") {
        FinishCalibration();
      }
    }
    // } else if (cmd == "start") {
    //   for (int l = 0; l < 4; ++l) {
    //     legs[l]->Attach();
    //   }
    // } else if (cmd == "ad") {
    //   FL.GetServo(2)->SetPosition(0.f);
    //   FL.GetServo(0)->SetPosition(0.f);
    //   FL.GetServo(1)->SetPosition(0.f);

    //   FR.GetServo(2)->SetPosition(0.f);
    //   FR.GetServo(0)->SetPosition(0.f);
    //   FR.GetServo(1)->SetPosition(0.f);

    //   BL.GetServo(2)->SetPosition(0.f);
    //   BL.GetServo(0)->SetPosition(0.f);
    //   BL.GetServo(1)->SetPosition(0.f);

    //   BR.GetServo(2)->SetPosition(0.f);
    //   BR.GetServo(0)->SetPosition(0.f);
    //   BR.GetServo(1)->SetPosition(0.f);

    // } else if (cmd == "au") {
    //   FL.GetServo(0)->SetPosition(-M_PI / 4);
    //   FL.GetServo(1)->SetPosition(M_PI / 4);

    //   FR.GetServo(0)->SetPosition(-M_PI / 4);
    //   FR.GetServo(1)->SetPosition(M_PI / 4);

    //   BL.GetServo(0)->SetPosition(-M_PI / 4);
    //   BL.GetServo(1)->SetPosition(M_PI / 4);

    //   BR.GetServo(0)->SetPosition(-M_PI / 4);
    //   BR.GetServo(1)->SetPosition(M_PI / 4);
    // }
  }
}
