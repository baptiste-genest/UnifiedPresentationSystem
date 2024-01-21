#ifndef UTILS_H
#define UTILS_H
#include "../UPS.h"

namespace UPS {
inline std::function<scalar(scalar)> buildRangeMapper(scalar a,scalar b,scalar c,scalar d) {
    return [a,b,c,d] (scalar x) {
        return c + (d-c)*(x-a)/(b-a);
    };
}
}

#endif // UTILS_H
