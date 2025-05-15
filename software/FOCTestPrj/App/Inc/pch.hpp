#ifndef __M_PCH_HPP__
#define __M_PCH_HPP__

extern "C" {
#pragma GCC diagnostic push
// NOLINTNEXTLINE
#pragma GCC diagnostic ignored "-Wvolatile"
#include "main.h"


#pragma GCC diagnostic pop
}  // extern "C"

class MainWidget {
 public:
  MainWidget();
  ~MainWidget();
  void show();
};

#endif /* __M_PCH_HPP__ */
