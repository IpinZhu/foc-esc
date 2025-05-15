#pragma once

#include "foc_utils.h"

using namespace MotorMath;

class MotorDriver {
public:
  MotorDriver();

  virtual void init();

  virtual void enable();
  virtual void disable();

  virtual void setPWMDuty(float duty, u8 channel);
};

/**
 * @brief 3 phase foc driver
 *
 */
class FOCDriver : MotorDriver {
public:
  FOCDriver() = default;

  void init() override;

  void enable() override;
  void disable() override;

  /**
   * @brief set the pwm duty
   *
   * @param duty 0-1
   * @return u8 (-1 : out of range; 1 : success)
   */
  void setPWMDuty(float duty, u8 channel) override;
  float getPWMDuty() const;
  // TODO:

  void setPhaseVoltage(float Ud, float Uq, float angle);

  void setPWMFrequency(long target_frequency);
  long getPWMFrequency() const;

private:
  long pwm_frequency = 30'000;
};