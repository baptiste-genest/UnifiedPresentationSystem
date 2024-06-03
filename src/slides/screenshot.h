#ifndef SCREENSHOT_H
#define SCREENSHOT_H


#ifdef __APPLE__
#include <string>
#else
#include "polyscope/screenshot.h"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#endif //__APPLE__

namespace slope {
  
  void screenshot(std::string file);
}
#endif // SCREENSHOT_H
