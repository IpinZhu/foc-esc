/**
 * @file ma_main.cpp
 * @author IpinZhu (github.com/IpinZhu)
 * @brief
 * @version 0.1
 * @date 2025-01-07
 *
 * @copyright Copyright (c) 2025 IpinZhu
 *
 */

#include <memory>
#include <stdio.h>

#include <format>
#include <string>

#include "brushless.h"
#include "pch.hpp"
#include "ustringart.h"

/** */
MainWidget::MainWidget() {
  // Initialize the widget
}
MainWidget::~MainWidget() {
  // Clean up the widget
}

void MainWidget::show() {
  // Show the widget
  StringUart Suart(true);
  while (1) {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    // Suart.UartOut("abcdefghijklmnopqrst\n");
  }
}

extern "C" {
// #include "arm_math.h"

void ma_main() {
  auto Motor{std::make_unique<CurrentController>};
  MainWidget widget;
  widget.show();
}

} // extern "C"