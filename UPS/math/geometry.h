#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "../UPS.h"

namespace UPS {

inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs)
{
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
inline ImVec2 ImRotate(const ImVec2& v, float cos_a, float sin_a)
{
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}
using mapping = std::function<vec(const vec&)>;

}


#endif // GEOMETRY_H
