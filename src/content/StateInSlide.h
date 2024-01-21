#ifndef STATEINSLIDE_H
#define STATEINSLIDE_H
#include "../UPS.h"
#include "io.h"
#include "Options.h"
#include "Referential.h"

namespace UPS {

struct StateInSlide {
    scalar alpha = 1;
    PositionPtr p;
    scalar angle=0;
    vec2 captured_pos;

    StateInSlide() {}

    StateInSlide(const vec2& x)  {
        p = std::make_shared<AbsolutePosition>(x);
    }

    StateInSlide& capturePos(const Slide& s) {
        captured_pos = getPosition(s);
        return *this;
    }

    vec2 getPosition(const Slide& s) const {
        if (p)
            return p->getPosition(s);
        return vec2(0,0);
    }
};

using StatePtr = std::shared_ptr<StateInSlide>;


}

#endif // STATEINSLIDE_H
