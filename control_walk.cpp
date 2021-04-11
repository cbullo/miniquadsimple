#include "control_walk.h"

void ControlWalk::Init() {
  auto* FL = GetActor()->GetLegFL();
  auto* FR = GetActor()->GetLegFR();
  auto* BL = GetActor()->GetLegBL();
  auto* BR = GetActor()->GetLegBR();

  neutral_position_[0] =
      FL->GetConfig().offset + Eigen::Vector3f(6.5f, 22.f, -60.f);

  neutral_position_[1] =
      FR->GetConfig().offset + Eigen::Vector3f(6.5f, -22.f, -60.f);

  neutral_position_[2] =
      BR->GetConfig().offset + Eigen::Vector3f(6.5f, -22.f, -60.f);

  neutral_position_[3] =
      BL->GetConfig().offset + Eigen::Vector3f(6.5f, 22.f, -60.f);

  for (int i = 0; i < 4; ++i) {
    leg_states_[i].pre_transform_position = neutral_position_[i];
    leg_states_[i].contact_active = true;
  }
}

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {}

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
}

Eigen::Vector3f ControlWalk::FindStepTarget(const Eigen::Vector3f& start_position,
                               int leg_index, const Steering& steering) {
  Eigen::Vector3f target =
      neutral_positions_[leg_index] +
      Eigen::Vector3f(0.5f * kStepMaxDistance * steering.forward, 0.f, 0.f);
}

void ControlWalk::UpdateNextStep(const Steering& steering) {
  auto next_step = FindLegForNextStep();
  if (next_step == -1) {
    return;
  }

  step_leg_index_ = next_step;

  step_start_contact_ = leg_states_[step_leg_index_].position;
  step_target_contact_ =
      FindStepTarget(step_start_contact, step_leg_index_, steering);

  auto target_pose_offset =
      (leg_states_[(step_leg_index_ + 3) % 4].effector_position +
       step_target_contact_ + leg_states_[(step_leg_index_ + 1) % 4]) /
      3.f;

  target_pose_ = Eigen::Translation(target_pose_offset);
}

float ControlWalk::PoseEaseInOut(float t) { return t; }

float ControlWalk::StepEaseInOut(float t) { return t; }

float ControlWalk::StepHeight(float t) const {
  return kStepHeight * sqrtf(1.f - (x - 1) * (x - 1));
}

void ControlWalk::ProgressStep(float delta_time) {
  step_t_ += delta_time * (1.f / kStepDuration);

  auto st = StepEaseInOut(step_t_);
  Vector3f height_v(0.f, 0.f, StepHeight(st));
  leg_states_[step_leg_index_].pre_transform_position =
      step_start_contact_ + st * (step_target_contact_ - step_start_contact_);

  auto t = PoseEaseInOut(step_t_);

  Eigen::Quaternion r_from = step_start_pose_.rotation();
  Eigen::Quaternion r_to = step_target_pose_.rotation();
  Eigen::Quaternion rotation = from.slerp(t, to);

  Eigen::Translation t_from = step_start_pose.translation();
  Eigen::Translation t_to = step_target_pose.translation();
  Eigen::Translation translation = t_from + t * (t_to - t_from);

  Eigen::Affine3f current_pose;
  current_pose.fromPositionOrientationScale(translation, rotation,
                                            Vector3f(1.f, 1.f, 1.f));

  for (int i = 0; i < 4; ++i) {
    leg_states_[i].transformed_position =
        current_pose * leg_states_[i].pre_transform_position;
    GetActor()->GetLegFixMe(i)->SetEffectorTarget(
        leg_states_[i].transformed_position);
  }
}