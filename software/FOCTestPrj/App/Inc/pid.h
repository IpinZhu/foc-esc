#pragma once

#include "foc_utils.h"

class PIDController {
public:
  /**
   * @brief Construct a new PIDController object
   *
   * @param P
   * @param I
   * @param D
   * @param ramp - Maximum speed of change of the output value
   * @param limit - Maximum output value
   */
  PIDController(float P, float I, float D, float ramp, float limit);
  ~PIDController() = default;

  void setParamater(float P, float I, float D, float ramp, float limit);

  float operator()(float error);
  void reset();

private:
  float m_P, m_I, m_D;
  float m_output_ramp; // Maximum speed of change of the output value
  float m_limit;       // Maximum output value

  float error_prev;             // last tracking error value
  float output_prev;            // last pid output value
  float integral_prev;          // last integral component value
  unsigned long timestamp_prev; // Last execution timestamp
};