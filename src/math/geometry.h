#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "../libslope.h"

namespace slope {

inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs)
{
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
inline ImVec2 ImRotate(const ImVec2& v, float cos_a, float sin_a)
{
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}
using mapping = std::function<vec(const vec&)>;


inline std::pair<vec,vec> CompleteBasis(const vec& x) {
    vec n = x.normalized();
    vec tmp(1.,0.,0.);
    if (std::abs(n[0]) > 0.999)
        tmp = vec(0.,1.,0.);
    vec a1 = n.cross(tmp);
    vec a2 = n.cross(a1);
    return {a1,a2};

}



}


#endif // GEOMETRY_H
