#include "control_walk.h"

ControlWalk::ControlWalk(Robot* robot) : ControlBase(robot) {
}

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
    leg_states_[i].pre_transform_position = neutral_positions_[i];
    leg_states_[i].contact_active = true;
  }

  elapsed_time = 0;

  step_leg_index_ = 0;
  step_t_ = 1.f;
}

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {
  
  if (elapsed_time > 10000) {
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
      return 3;
    case 3:
      return 1;
    case 1:
      return 4;
    case 4:
      return 0;
  }

  return 0;
}

Eigen::Vector3f ControlWalk::FindStepTarget(
    const Eigen::Vector3f& start_position, int leg_index,
    const Steering& steering) {
  Eigen::Vector3f target =
      neutral_positions_[leg_index] +
      Eigen::Vector3f(0.5f * kStepMaxDistance * steering.forward, 0.f, 0.f);

  return target;
}

void ControlWalk::UpdateNextStep(const Steering& steering) {
  auto next_step = FindLegForNextStep(step_leg_index_);
  if (next_step == -1) {
    step_leg_index_ = -1;
    return;
  }

  step_leg_index_ = next_step;
  Serial.printf("Next step: %d\n", step_leg_index_);

  step_start_contact_ = leg_states_[step_leg_index_].pre_transform_position;
  step_target_contact_ =
      FindStepTarget(step_start_contact_, step_leg_index_, steering);

  auto target_pose_offset =
      (leg_states_[(step_leg_index_ + 3) % 4].pre_transform_position +
       step_target_contact_ +
       leg_states_[(step_leg_index_ + 1) % 4].pre_transform_position) /
      3.f;

  step_start_pose_ = step_target_pose_;
  step_target_pose_ = Eigen::Translation3f(target_pose_offset);
  step_t_ = 0.f;
}

float ControlWalk::PoseEaseInOut(float t) { return t; }

float ControlWalk::StepEaseInOut(float t) { return t; }

float ControlWalk::StepHeight(float t) const {
  return kStepHeight * sqrtf(1.f - (t - 1) * (t - 1));
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
  leg_states_[step_leg_index_].pre_transform_position =
      step_start_contact_ + st * (step_target_contact_ - step_start_contact_);

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
    leg_states_[i].transformed_position =
        current_pose * leg_states_[i].pre_transform_position;
    GetActor()->GetLegFixMe(i)->SetEffectorTarget(
        leg_states_[i].transformed_position);
    Serial.printf("Leg(%d) target: %f %f %f\n", i,
                  leg_states_[i].pre_transform_position(0),
                  leg_states_[i].pre_transform_position(1),
                  leg_states_[i].pre_transform_position(2));
  }

  return finish_step;
}