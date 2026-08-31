#pragma once

#include "foc_driver.h"
#include "foc_utils.h"
#include "pid.h"
#include "sensor.h"
#include "ustringart.h"
#include <array>
#include <vector>

class FOCMotor {
public:
  /**
   * @brief Construct a new FOCMotor object
   *
   * @param pp pole pairs number
   * @param R resistance
   * @param KV KV rating
   * @param L inductance
   */
  FOCMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET);

  /**
   * @brief initializing FOC algorithm
   *
   */
  void init();

  /**
   * @brief linking a foc driver for pwm output
   *
   * @param driver FOCDriver class
   */
  void linkDriver(FOCDriver *driver);

  /**
   * @brief linking a sensor for angle
   *
   */
  void linkSensor(FOCSensor *sensor);

  /**
   * @brief Function running FOC algorithm in real-time
   *
   */
  void loopFOC();

  void setAngle();

  void setDCVoltage();
  void setVelocity();

private:
  FOCDriver *m_driver;
  FOCSensor *m_sensor;

  PIDController current_PID, velocity_PID;

  u8 pole_pairs;
  float phase_resistance; // ohm
  float kv_rating;
  float inductance;

  float Ua, Ub, Uc;

  /**
   * @brief open loop controller
   *
   * @param target_velocity
   * @return float
   */
  float velocityOpenloop(float target_velocity);

  float angleOpenloop(float target_angle);
};