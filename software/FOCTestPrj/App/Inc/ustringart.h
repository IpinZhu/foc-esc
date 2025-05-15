/**
 * @file ustringart.h
 * @author IpinZhu (github.com/IpinZhu)
 * @brief
 * @version 0.1
 * @date 2025-01-08
 *
 * @copyright Copyright (c) 2025 IpinZhu
 *
 */
#ifndef __USTRINGART_H
#define __USTRINGART_H

#include <cstring>
#include <string>


#include "pch.hpp"

class StringUart {
 public:
  StringUart(bool isdma);
  ~StringUart() = default;

  void UartOut(const std::string &buff);
  std::string UartIn() const;

 protected:
 private:
  void UartTransmit(const unsigned char *buff, unsigned int size) const;
  void UartInput() const;
  bool is_dma;
  const unsigned int timeout = 1000;
};

#endif