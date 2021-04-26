#include "control_walk.h"

ControlWalk::ControlWalk(Robot* robot) : ControlBase(robot) {}

void ControlWalk::Init() {
  auto* FL = GetActor()->GetLegFL();
  auto* FR = GetActor()->GetLegFR();
  auto* BL = GetActor()->GetLegBL();
  auto* BR = GetActor()->GetLegBR();

  neutral_positions_[0] =
      FL->GetConfig().offset + Eigen::Vector3f(6.5f, 15.f, -65.f);

  neutral_positions_[1] =
      FR->GetConfig().offset + Eigen::Vector3f(6.5f, -15.f, -65.f);

  neutral_positions_[2] =
      BR->GetConfig().offset + Eigen::Vector3f(6.5f, -15.f, -65.f);

  neutral_positions_[3] =
      BL->GetConfig().offset + Eigen::Vector3f(6.5f, 15.f, -65.f);

  for (int i = 0; i < 4; ++i) {
    pre_transform_position[i] = neutral_positions_[i];
  }

  elapsed_time = 0;
  step_t_ = 0.f;
}

void ControlWalk::ProcessInput(float axes[6], uint32_t buttons) {
  if (elapsed_time >= 5000) {
    Steering steering;
    steering.forward = 2.f * axes[1] - 1.f;
    if (fabsf(steering.forward) < 0.1f) {
      steering.forward = 0.f;
    }

    ProgressStep(0.005f, steering);

    elapsed_time -= 5000;
  }
}

Eigen::Vector3f ControlWalk::GetStepOffset(float t, int leg_index) {
  float l = 0.f;
  float h = 0.f;

  float s = steps_[leg_index].start_offset;
  float e = s + kStepDuration / kCycleDuration;
  float r = s - e + 1;

  if (t < s) {
    float t1 = (t - e + 1.f) / r;
    l = (0.5 - t1) * kStepMaxDistance;
    h = 0.f;
  } else if (t > e) {
    float t1 = (t - e) / r;
    l = (0.5 - t1) * kStepMaxDistance;
    h = 0.f;
  } else {
    float st = (t - s) / (e - s);
    l = (-0.5f + 1.f * st) * kStepMaxDistance;
    h = StepHeight(st);
  }

  return Eigen::Vector3f(l, 0.f, h);
}

float ControlWalk::PoseEaseInOut(float t) { return t; }

float ControlWalk::StepEaseInOut(float t) { return t; }

float ControlWalk::StepHeight(float t) const {
  if (t <= 0 || t >= 1.f) {
    return 0.f;
  }

  return kStepMaxHeight * sqrtf(1.f - (2.f * t - 1) * (2.f * t - 1));
}

bool ControlWalk::ProgressStep(float delta_time, const Steering& steering) {
  step_t_ += delta_time * (1.f / kCycleDuration);

  // if (step_t_ >= 1.f) {
  //   current_step_distance_ = kStepMinDistance + fabsf(steering.forward) * (kStepMaxDistance - kStepMinDistance);
  //   current_step_height_ = 
  // }
  step_t_ = fmodf(step_t_, 1.f);

  //float step_distance = 

  for (int i = 0; i < 4; ++i) {
    pre_transform_position[i] =
        neutral_positions_[i] + GetStepOffset(step_t_, i);
  }

  bool interpolating = false;
  for (int i = 0; i < 4; ++i) {
    if (step_t_ >= poses_[i].start_offset &&
        step_t_ <= poses_[i].start_offset + poses_[i].cycle_fraction) {
      float pt = (step_t_ - poses_[i].start_offset) / poses_[i].cycle_fraction;
      auto target = poses_[i].target;

      step_target_pose_ = Eigen::Translation3f(target);

      Eigen::Quaternionf r_from(step_start_pose_.rotation());
      Eigen::Quaternionf r_to(step_target_pose_.rotation());
      Eigen::Quaternionf rotation = r_from.slerp(pt, r_to);

      Eigen::Vector3f t_from(step_start_pose_.translation());
      Eigen::Vector3f t_to(step_target_pose_.translation());
      Eigen::Vector3f translation = t_from + pt * (t_to - t_from);

      current_pose_.fromPositionOrientationScale(
          translation, rotation, Eigen::Vector3f(1.f, 1.f, 1.f));
      interpolating = true;
    }
  }

  if (!interpolating) {
    step_start_pose_ = current_pose_;
  }

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

  return true;
}