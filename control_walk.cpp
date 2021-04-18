#include "control_walk.h"

ControlWalk::ControlWalk(Robot* robot) : ControlBase(robot) {}

void ControlWalk::Init() {
  auto* FL = GetActor()->GetLegFL();
  auto* FR = GetActor()->GetLegFR();
  auto* BL = GetActor()->GetLegBL();
  auto* BR = GetActor()->GetLegBR();

  neutral_positions_[0] =
      FL->GetConfig().offset + Eigen::Vector3f(6.5f, 15.f, -70.f);

  neutral_positions_[1] =
      FR->GetConfig().offset + Eigen::Vector3f(6.5f, -15.f, -70.f);

  neutral_positions_[2] =
      BR->GetConfig().offset + Eigen::Vector3f(6.5f, -15.f, -60.f);

  neutral_positions_[3] =
      BL->GetConfig().offset + Eigen::Vector3f(6.5f, 15.f, -60.f);

  for (int i = 0; i < 4; ++i) {
    pre_transform_position[i] = neutral_positions_[i];
    steps[i].step_start = neutral_positions_[i];
    // leg_states_[i].contact_active = true;
  }

  elapsed_time = 0;

  step_leg_index_ = 0;
  step_t_ = 0.f;

  step_start_contact_ = neutral_positions_[0];
  step_target_contact_ = neutral_positions_[0];
}

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {
  if (elapsed_time >= 1000) {
    Steering steering;
    steering.forward = axes[1];

    if (ProgressStep(0.001f)) {
      UpdateNextStep(steering);
    }

    elapsed_time -= 1000;
  }
}

int ControlWalk::FindLegForNextStep(int previous_step) const {
  switch (previous_step) {
    case 0:
      return 3;
    case 3:
      return 1;
    case 1:
      return 2;
    case 2:
      return 0;
  }

  return 0;
}

Eigen::Vector3f ControlWalk::FindStepTarget(
    const Eigen::Vector3f& start_position, int leg_index,
    const Steering& steering) {
  Eigen::Vector3f target =
      neutral_positions_[leg_index] +
      Eigen::Vector3f(0.5f * kStepMaxDistance /** steering.forward*/, 0.f, 0.f);

  return target;
}

void ControlWalk::UpdateNextStep(const Steering& steering) {
  auto next_step = FindLegForNextStep(step_leg_index_);
  if (next_step == -1) {
    step_leg_index_ = -1;
    return;
  }

  step_leg_index_ = next_step;

  step_start_contact_ = pre_transform_position[step_leg_index_];
  step_target_contact_ =
      FindStepTarget(step_start_contact_, step_leg_index_, steering);

  Eigen::Vector3f target_pose_offset =
      (pre_transform_position[(step_leg_index_ + 1) % 4] +
       pre_transform_position[(step_leg_index_ + 2) % 4] +
       pre_transform_position[(step_leg_index_ + 3) % 4]) /
      3.f;

  target_pose_offset(0) = 0.f;
  target_pose_offset(2) = 0.f;

  // step_start_pose_ = step_target_pose_;
  // step_target_pose_ = Eigen::Translation3f(target_pose_offset);

  step_t_ = 0.f;
  for (int i = 0; i < 4; ++i) {
    steps[i].step_start = pre_transform_position[i];
  }

  // Serial.printf("Next step: %d, step_target.x: %f, target_pose_offset.x:
  // %f\n",
  //               step_leg_index_, step_target_contact_(0),
  //               target_pose_offset(0));
}

float ControlWalk::PoseEaseInOut(float t) { return t; }

float ControlWalk::StepEaseInOut(float t) { return t; }

Eigen::Vector3f ControlWalk::GetLegOffset(int leg_index, float t) {
  float x = 0.f;
  float z = 0.f;

  float back_fraction = 1.f - steps[i].cycle_fraction;

  if (t >= steps[i].start_offset &&
      t <= steps[i].start_offset + steps[i].cycle_fraction) {
    float st = (t - steps[i].start_offset) / steps[i].cycle_fraction;
    z = StepHeight(st);
    x = (st - 0.5f) * kStepMaxDistance;
  } else {
    if (t < steps[i].start_offset) {
    } else if (t > steps[i].start_offset + steps[i].cycle_fraction) {
    }
  }

  float ControlWalk::StepHeight(float t) const {
    if (t <= 0 || t >= 1.f) {
      return 0.f;
    }

    return kStepHeight * sqrtf(1.f - (2.f * t - 1) * (2.f * t - 1));
  }

  bool ControlWalk::ProgressStep(float delta_time) {
    if (step_leg_index_ == -1) {
      return false;
    }

    step_t_ += delta_time * (1.f / kCycleDuration);

    bool finish_step = false;
    if (step_t_ >= 1.f) {
      step_t_ = 1.f;
      finish_step = true;
    }

    bool printme = false;
    for (int i = 0; i < 4; ++i) {
      if (step_t_ >= steps[i].start_offset &&
          step_t_ <= steps[i].start_offset + steps[i].cycle_fraction) {
        float st = (step_t_ - steps[i].start_offset) / steps[i].cycle_fraction;
        Eigen::Vector3f height_offset(0.f, 0.f, StepHeight(st));
        Eigen::Vector3f step_offset =
            0.5f * Eigen::Vector3f(kStepMaxDistance, 0.f, 0.f);
        Eigen::Vector3f target = neutral_positions_[i] + step_offset;

        if (i == 2) {
          Serial.printf("Neutral: %f %f %f\n", neutral_positions_[2](0),
                        neutral_positions_[2](1), neutral_positions_[2](2));
          Serial.printf("Target: %f\n", target(2));
        }

        pre_transform_position[i] = steps[i].step_start +
                                    st * (target - steps[i].step_start) +
                                    height_offset;
        if (i == 2) {
          Serial.printf("Neutral: %f %f %f\n", neutral_positions_[2](0),
                        neutral_positions_[2](1), neutral_positions_[2](2));
          Serial.printf("Target: %f, st: %f, pre_transform_position: %f\n",
                        target(2), st, pre_transform_position[2](2));
          printme = true;
        }

      } else {
        pre_transform_position[i](2) = neutral_positions_[i](2);
        steps[i].step_start = pre_transform_position[i];
      }

      pre_transform_position[i](0) =
          pre_transform_position[i](0) -
          (delta_time * (1.f / kCycleDuration)) * kStepMaxDistance * 2.f;
    }

    auto t = PoseEaseInOut(step_t_);

    bool interpolating = false;
    for (int i = 0; i < 4; ++i) {
      if (step_t_ >= poses[i].start_offset &&
          step_t_ <= poses[i].start_offset + poses[i].cycle_fraction) {
        float pt = (step_t_ - poses[i].start_offset) / poses[i].cycle_fraction;
        auto target = poses[i].target;

        step_target_pose_ = Eigen::Translation3f(target);

        Eigen::Quaternionf r_from(step_start_pose_.rotation());
        Eigen::Quaternionf r_to(step_target_pose_.rotation());
        Eigen::Quaternionf rotation = r_from.slerp(t, r_to);

        Eigen::Vector3f t_from(step_start_pose_.translation());
        Eigen::Vector3f t_to(step_target_pose_.translation());
        Eigen::Vector3f translation = t_from + t * (t_to - t_from);

        current_pose_.fromPositionOrientationScale(
            translation, rotation, Eigen::Vector3f(1.f, 1.f, 1.f));
        interpolating = true;
      }
    }

    if (!interpolating) {
      step_start_pose_ = current_pose_;
    }

    if (printme)
      Serial.printf("pre_transform_position: %f\n",
                    pre_transform_position[2](2));

    for (int i = 0; i < 4; ++i) {
      transformed_position[i] =
          current_pose_.inverse() * pre_transform_position[i];

      // Serial.printf("Leg(%d) target: %f %f %f, step_t: %f, step_leg_index:
      // %d\n",
      //               i, transformed_position[i](0),
      //               transformed_position[i](1), transformed_position[i](2),
      //               step_t_, step_leg_index_);

      GetActor()->GetLegFixMe(i)->SetEffectorTarget(transformed_position[i]);
    }

    return finish_step;
  }