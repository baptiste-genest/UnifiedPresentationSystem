#ifndef UTILS_H
#define UTILS_H
#include "../UPS.h"

namespace UPS {

inline std::function<scalar(scalar)> buildRangeMapper(scalar a,scalar b,scalar c,scalar d) {
    return [a,b,c,d] (scalar x) {
        return c + (d-c)*(x-a)/(b-a);
    };
}

inline vec2 lerp(const vec2& a,const vec2& b,scalar t) {
    return a + (b-a)*t;
}

}

#endif // UTILS_H
