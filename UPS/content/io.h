#ifndef IO_H
#define IO_H
#include "../UPS.h"

#include <fstream>

namespace UPS {

namespace io {

inline bool file_exists(std::string filename) {
    return std::ifstream(filename).good();
}

}

}

#endif // IO_H
