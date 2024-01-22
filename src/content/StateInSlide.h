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

    StateInSlide() {}

    StateInSlide(const vec2& x)  {
        p = std::make_shared<AbsolutePosition>(x);
    }

    vec2 getPosition() const {
        if (p)
            return p->getPosition();
        return vec2(0,0);
    }

    ImVec2 getAbsolutePosition() const {
        vec2 P = getPosition();
        auto W = ImGui::GetWindowSize();
        return ImVec2(P(0)*W.x,P(1)*W.y);
    }
};

using StatePtr = std::shared_ptr<StateInSlide>;


}

#endif // STATEINSLIDE_H
