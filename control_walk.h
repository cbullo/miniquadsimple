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
  bool ProgressStep(float delta_time, const Steering& steering);

 private:
  struct StepDefinition {
    float start_offset;
  };

  struct PoseDefinition {
    float start_offset;
    float cycle_fraction;
    Eigen::Vector3f target;
  };

  StepDefinition steps_[4] = {{0.0}, {0.5}, {0.0}, {0.5}};

  PoseDefinition poses_[4] = {{0.0, 0.1, Eigen::Vector3f(0, 0, 0)},
                              {0.25, 0.25, Eigen::Vector3f(0, 0, 0)},
                              {0.5, 0.1, Eigen::Vector3f(0, -0, 0)},
                              {0.75, 0.25, Eigen::Vector3f(-0, -0, 0)}};

  struct LegState {
    Eigen::Vector3f current_position;
    Eigen::Vector3f last_contact_position;
    Eigen::Vector3f step_target;
  };
  LegState states_[4];

  Eigen::Vector3f GetStepPosition(float t, float delta_t, int leg_index,
                                  const Steering& steering);
  void UpdateNextStep(const Steering& steering);

  static float PoseEaseInOut(float t);
  static float StepEaseInOut(float t);
  float StepHeight(float t) const;

  const float kStepMinHeight = 15.f;
  const float kStepMaxHeight = 25.f;
  const float kStepMinDistance = 10.f;
  const float kStepMaxDistance = 55.f;
  const float kCycleDuration = 1.5f;
  const float kStepDuration = 0.15f;

  float step_t_ = 0.f;  // Step progress 0.f to 1.f

  Eigen::Vector3f neutral_positions_[4];

  Eigen::Affine3f step_start_pose_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f step_target_pose_ = Eigen::Affine3f::Identity();
  Eigen::Affine3f current_pose_ = Eigen::Affine3f::Identity();

  Eigen::Vector2f steering_;

  elapsedMicros elapsed_time;
  Eigen::Vector3f pre_transform_position[4];
  Eigen::Vector3f transformed_position[4];
};