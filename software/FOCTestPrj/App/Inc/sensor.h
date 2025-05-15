#pragma once

class FOCSensor {
public:
  FOCSensor();

  float getAngle() const;

  float getVeloctiy() const;

  void update();

private:
  /**
   * @brief to calculate the velocity, default s
   *
   */
  float elapsed_time = 0.000100f;
  float angle = 0.0f;
  float angle_prev = 0.0f;
  float velocity = 0.0f;
  float velocity_average = 0.0f;
};