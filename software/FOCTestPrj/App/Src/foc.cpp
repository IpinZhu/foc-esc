/**
 * @file foc.cpp
 * @author IpinZhu (github.com/IpinZhu)
 * @brief foc api
 * @version 0.1
 * @date 2025-03-18
 *
 * @copyright Copyright (c) 2025 IpinZhu
 *
 */
#include "foc.h"

FOCMotor::FOCMotor(int pp, float R, float KV, float L) : m_debugger(0) {}

void FOCMotor::init() {
  m_driver->setDCVoltage(12.0f);
  m_driver->setPWMFrequency(30'000);
  m_driver->enable();
  m_driver->setPWM(0.0f, 0.0f, 0.0f);
}