#include "control_walk.h"

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {}

int ControlWalk::FindLegForNextStep() const {
  if (step_leg_index_ != -1) {
    return -1;
  }
}

Eigen::Vector3f FindStepTarget(const Eigen::Vector3f& start_position,
                               int leg_index, const Steering& steering) {
  Eigen::Vector3f target =
      neutral_positions_[leg_index] +
      Eigen::Vector3f(kStepMaxDistance * steering.forward, 0.f, 0.f);
}

void UpdateNextStep(const Steering& steering) {
  auto next_step = FindLegForNextStep();
  if (next_step == -1) {
    return;
  }

  step_leg_index_ = next_step;

  step_start_contact_ = contacts_[step_leg_index_].position;
  step_target_contact_ =
      FindStepTarget(step_start_contact, step_leg_index_, steering);

  auto target_pose_offset_ =
      (contacts_[(step_leg_index_ + 3) % 4] + step_target_contact_ +
       contacts_[(step_leg_index_ + 1) % 4]) /
      3.f;

  target_pose_ = Eigen::Translation(target_pose_offset_);
}
void ProgressStep(float delta_time);