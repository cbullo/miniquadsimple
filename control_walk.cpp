#include "control_walk.h"

ControlWalk::ControlWalk(Robot* robot) : ControlBase(robot) {}

void ControlWalk::Init() {
  auto* FL = GetActor()->GetLegFL();
  auto* FR = GetActor()->GetLegFR();
  auto* BL = GetActor()->GetLegBL();
  auto* BR = GetActor()->GetLegBR();

  neutral_positions_[0] =
      FL->GetConfig().offset + Eigen::Vector3f(6.5f, 22.f, -60.f);

  neutral_positions_[1] =
      FR->GetConfig().offset + Eigen::Vector3f(6.5f, -22.f, -60.f);

  neutral_positions_[2] =
      BR->GetConfig().offset + Eigen::Vector3f(6.5f, -22.f, -60.f);

  neutral_positions_[3] =
      BL->GetConfig().offset + Eigen::Vector3f(6.5f, 22.f, -60.f);

  for (int i = 0; i < 4; ++i) {
    pre_transform_position[i] = neutral_positions_[i];
    // leg_states_[i].contact_active = true;
  }

  elapsed_time = 0;

  step_leg_index_ = 0;
  step_t_ = 0.f;

  step_start_contact_ = neutral_positions_[0];
  step_target_contact_ = neutral_positions_[0];
}

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {
  if (elapsed_time >= 10000) {
    Steering steering;
    steering.forward = axes[1];

    if (ProgressStep(0.01f)) {
      UpdateNextStep(steering);
    }

    elapsed_time -= 10000;
  }
}

int ControlWalk::FindLegForNextStep(int previous_step) const {
  switch (previous_step) {
    case 0:
      return 2;
    case 2:
      return 1;
    case 1:
      return 3;
    case 3:
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
  target_pose_offset(2) = 0.f;

  step_start_pose_ = step_target_pose_;
  step_target_pose_ = Eigen::Translation3f(target_pose_offset);
  step_t_ = 0.f;

  // Serial.printf("Next step: %d, step_target.x: %f, target_pose_offset.x:
  // %f\n",
  //               step_leg_index_, step_target_contact_(0),
  //               target_pose_offset(0));
}

float ControlWalk::PoseEaseInOut(float t) {
  if (t < 0.5) {
    return 2 * t;
  } else {
    return 1.f;
  }
}

float ControlWalk::StepEaseInOut(float t) {
  if (t < 0.5) {
    return 0.f;
  } else {
    return 2.f * (t - 0.5f);
  }
}

float ControlWalk::StepHeight(float t) const {
  return kStepHeight * sqrtf(1.f - (2.f * t - 1) * (2.f * t - 1));
}

bool ControlWalk::ProgressStep(float delta_time) {
  if (step_leg_index_ == -1) {
    return false;
  }

  step_t_ += delta_time * (1.f / kStepDuration);

  bool finish_step = false;
  if (step_t_ >= 1.f) {
    step_t_ = 1.f;
    finish_step = true;
  }

  auto st = StepEaseInOut(step_t_);
  Eigen::Vector3f height_v(0.f, 0.f, StepHeight(st));
  pre_transform_position[step_leg_index_] =
      step_start_contact_ + st * (step_target_contact_ - step_start_contact_) +
      height_v;

  if (step_t_ > 0.5f) {
    for (int i = 0; i < 4; ++i) {
      // if (i == step_leg_index_) {
      //   continue;
      // }
      pre_transform_position[i](0) =
          pre_transform_position[i](0) -
          (delta_time * (1.f / kStepDuration) * 0.25f) * kStepMaxDistance * 2.f;
    }
  }

  auto t = PoseEaseInOut(step_t_);

  Eigen::Quaternionf r_from(step_start_pose_.rotation());
  Eigen::Quaternionf r_to(step_target_pose_.rotation());
  Eigen::Quaternionf rotation = r_from.slerp(t, r_to);

  Eigen::Vector3f t_from(step_start_pose_.translation());
  Eigen::Vector3f t_to(step_target_pose_.translation());
  Eigen::Vector3f translation = t_from + t * (t_to - t_from);

  Eigen::Affine3f current_pose;
  current_pose.fromPositionOrientationScale(translation, rotation,
                                            Eigen::Vector3f(1.f, 1.f, 1.f));

  for (int i = 0; i < 4; ++i) {
    transformed_position[i] =
        current_pose.inverse() * pre_transform_position[i];

    // transformed_position[i] = pre_transform_position[i];
    // Serial.printf("Leg(%d) target: %f %f %f, step_t: %f, step_leg_index: %d\n",
    //               i, transformed_position[i](0), transformed_position[i](1),
    //               transformed_position[i](2), st, step_leg_index_);

    GetActor()->GetLegFixMe(i)->SetEffectorTarget(
      transformed_position[i]);
  }

  return finish_step;
}