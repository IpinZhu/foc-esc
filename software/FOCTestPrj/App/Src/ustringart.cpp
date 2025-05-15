/**
 * @file ustringart.cpp
 * @author IpinZhu (github.com/IpinZhu)
 * @brief
 * @version 0.1
 * @date 2025-01-08
 *
 * @copyright Copyright (c) 2025 IpinZhu
 *
 */

#include "ustringart.h"
extern "C" {
extern uint8_t buffe[1024];

extern UART_HandleTypeDef huart1;
StringUart::StringUart(bool dma) {
  is_dma = dma;
  if (dma) {
    // m_dmabuffer = new unsigned char[1024]() ;
  } else {
    return;
  }
};

void StringUart::UartOut(const std::string &buff) {
  char *cstr;
  unsigned int lenths{buff.size()};
  cstr = new char[lenths + 1]();

  if (is_dma) {
    strcpy(cstr, buff.c_str());
    memcpy(buffe, cstr, lenths + 1);
    UartTransmit(buffe, lenths + 1);
    delete[] cstr;

  } else {
    unsigned char *ucstr;
    ucstr = new unsigned char[lenths + 1]();
    strcpy(cstr, buff.c_str());
    memcpy(ucstr, cstr, lenths + 1);
    UartTransmit(ucstr, lenths + 1);
    delete[] cstr;
    delete[] ucstr;
  }
}

void StringUart::UartTransmit(const unsigned char *buff,
                              unsigned int size) const {
  if (is_dma) {
    HAL_UART_Transmit_DMA(&huart1, buff, size);
  } else {
    HAL_UART_Transmit(&huart1, buff, size, 100);
  }
}
}