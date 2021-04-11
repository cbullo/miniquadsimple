#pragma once

#include "control.h"

class ControlWalk : ControlBase {
 public:
  Control2DSideIK(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;

  int FindLegForNextStep() const;
  void UpdateNextStep();
  void ProgressStep(float delta_time);

 private:
  struct LegState {
    Eigen::Vector3f effector_position;
    bool contact_active;
  };

  const float kMaxStepDelta = 0.1f;
  const float kStepHeight = 10.f;
  const float kStepMaxDistance = 20.f;

  float step_t_;  // Step progress 0.f to 1.f
  int step_leg_index_ = -1; // -1 - step inactive
  Eigen::Vector3f step_start_contact_;
  Eigen::Vector3f step_target_contact_;

  Contact leg_states_[4];

  Eigen::Affine3f step_start_pose_;
  Eigen::Affine3f target_pose_;

  Eigen::Vector2f steering_;
};