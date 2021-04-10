#pragma once

#include "Eigen.h"
#include "leg.h"

class Robot {
 public:
  void Init(LegConfig* FL, LegConfig* FR, LegConfig* BL, LegConfig* BR);
  void ApplyConfig();

  void SetTiltAndAngle(const Eigen::Vector3f& tilt, float swing);
  void SetOffset(const Eigen::Vector3f& offset);
  void SetAngle(int leg_index, float front_servo, float back_servo,
                float side_servo);

  Leg* GetLegFL() { return &FL; }
  Leg* GetLegFR() { return &FR; }
  Leg* GetLegBL() { return &BL; }
  Leg* GetLegBR() { return &BR; }
  Leg* GetLeg(int leg_index) { return legs[leg_index]; }

  const Leg* GetLegFL() const { return &FL; }
  const Leg* GetLegFR() const { return &FR; }
  const Leg* GetLegBL() const { return &BL; }
  const Leg* GetLegBR() const { return &BR; }
  const Leg* GetLeg(int leg_index) const { return legs[leg_index]; }

 private:
  Leg FL;
  Leg FR;
  Leg BL;
  Leg BR;
  Leg* legs[4] = {&FL, &FR, &BL, &BR};
};

class ControlBase {
 public:
  ControlBase(Robot* actor);
  virtual void ProcessInput(float axes[6], uint32_t buttons) = 0;
  Robot* GetActor() const { return actor_; }

 private:
  Robot* actor_;
};

class ControlOffset : public ControlBase {
 public:
  ControlOffset(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;
};

class ControlAngles : public ControlBase {
 public:
  ControlAngles(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;
};

class Control2DIK : public ControlBase {
 public:
  Control2DIK(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;
};

class Control2DSideIK : public ControlBase {
 public:
  Control2DSideIK(Robot* actor);
  void ProcessInput(float axes[6], uint32_t buttons) override;
};