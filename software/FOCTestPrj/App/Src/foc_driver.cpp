/**
 * @file foc_driver.cpp
 * @author IpinZhu (github.com/IpinZhu)
 * @brief
 * @version 0.1
 * @date 2025-03-18
 *
 * @copyright Copyright (c) 2025 IpinZhu
 *
 */

#include "foc_driver.h"
#include <sys/_intsup.h>

extern "C" {

void FOCDriver::init() {}

void FOCDriver::enable() {}

void FOCDriver::disable() {}

void FOCDriver::setPWMDuty(float duty, u8 channel) {}

float FOCDriver::getPWMDuty() const {}

void FOCDriver::setPhaseVoltage(float Ud, float Uq, float angle) {

  float phase_a = Ud * cos(angle) - Uq * sin(angle);
  float phase_b = Ud * sin(angle) + Uq * cos(angle);
  float phase_c = -phase_a - phase_b;

  setPWMDuty(phase_a, 1);
  setPWMDuty(phase_b, 2);
  setPWMDuty(phase_c, 3);
}

void FOCDriver::setPWMFrequency(long target_frequency) {}

long FOCDriver::getPWMFrequency() const { return pwm_frequency; }
}
