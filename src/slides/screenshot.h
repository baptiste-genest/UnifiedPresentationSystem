#ifndef SCREENSHOT_H
#define SCREENSHOT_H

#include "polyscope/screenshot.h"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

namespace UPS {

void screenshot(std::string file);
}

#endif // SCREENSHOT_H
