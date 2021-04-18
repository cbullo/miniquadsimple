#pragma once

#include "Arduino.h"
#include "Eigen.h"
#include "Eigen/Geometry"
#include "control.h"

struct Steering {
  float forward = 0.f;
  float side = 0.f;
  float turn = 0.f;
};

class ControlWalk : public ControlBase {
 public:
  ControlWalk(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;

  void Init();

  int FindLegForNextStep(int previous_step) const;
  void UpdateNextStep();
  bool ProgressStep(float delta_time);

 private:
  struct StepDefinition {
    float start_offset;
    float cycle_fraction;
    Eigen::Vector3f step_start;
  };

  struct PoseDefinition {
    float start_offset;
    float cycle_fraction;
    Eigen::Vector3f target;
  };

  StepDefinition steps[4] = {{0.0, 0.2}, {0.5, 0.2}, {0.0, 0.2}, {0.5, 0.2}};

  PoseDefinition poses[4] = {{0.0, 0.1, Eigen::Vector3f(0, 0, 0)},
                             {0.25, 0.25, Eigen::Vector3f(0, 0, 0)},
                             {0.5, 0.1, Eigen::Vector3f(0, -0, 0)},
                             {0.75, 0.25, Eigen::Vector3f(-0, -0, 0)}};

  Eigen::Vector3f FindStepTarget(const Eigen::Vector3f& start_position,
                                 int leg_index, const Steering& steering);
  void UpdateNextStep(const Steering& steering);

  static float PoseEaseInOut(float t);
  static float StepEaseInOut(float t);
  float StepHeight(float t) const;

  // struct LegState {

  // bool contact_active;
  //};

  const float kStepHeight = 35.f;
  const float kStepMaxDistance = 25.f;
  const float kCycleDuration = 0.6f;

  float step_t_ = 0.f;      // Step progress 0.f to 1.f
  int step_leg_index_ = 0;  // -1 - step inactive
  Eigen::Vector3f step_start_contact_;
  Eigen::Vector3f step_target_contact_;

  // LegState leg_states_[4];
  Eigen::Vector3f neutral_positions_[4];

  Eigen::Affine3f step_start_pose_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f step_target_pose_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f current_pose_ = Eigen::Affine3f::Identity();


  Eigen::Vector2f steering_;

  elapsedMicros elapsed_time;
  Eigen::Vector3f pre_transform_position[4];
  Eigen::Vector3f transformed_position[4];
};