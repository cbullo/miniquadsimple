#pragma once

#include "Eigen.h"
#include "Eigen/Geometry"
#include "control.h"

class ControlWalk : ControlBase {
 public:
  ControlWalk(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;

  void Init();

  int FindLegForNextStep(int previous_step) const;
  void UpdateNextStep();
  void ProgressStep(float delta_time);

 private:
  Eigen::Vector3f FindStepTarget(const Eigen::Vector3f& start_position,
                                 int leg_index, const Steering& steering);
  void UpdateNextStep(const Steering& steering);

  static float PoseEaseInOut(float t);
  static float StepEaseInOut(float t);
  static float StepHeight(float t);

  struct LegState {
    Eigen::Vector3f pre_transform_position;
    Eigen::Vector3f transformed_position;
    bool contact_active;
  };

  const float kMaxStepDelta = 0.1f;
  const float kStepHeight = 10.f;
  const float kStepMaxDistance = 20.f;
  const float kStepDuration = 1.f;

  float step_t_;             // Step progress 0.f to 1.f
  int step_leg_index_ = -1;  // -1 - step inactive
  Eigen::Vector3f step_start_contact_;
  Eigen::Vector3f step_target_contact_;

  LegState leg_states_[4];
  Eigen::Vector3f neutral_position_[4];

  Eigen::Affine3f step_start_pose_;
  Eigen::Affine3f step_target_pose_;

  Eigen::Vector2f steering_;
};