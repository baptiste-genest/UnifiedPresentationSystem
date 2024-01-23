#ifndef STATEINSLIDE_H
#define STATEINSLIDE_H
#include "../UPS.h"
#include "io.h"
#include "Options.h"
#include "Anchor.h"

namespace UPS {

using RelativePlacer = std::function<vec2(vec2)>;

struct StateInSlide {
    scalar alpha = 1;
    RelativePlacer placer = [] (const vec2& p) {
        return p;
    };
    AnchorPtr anchor = GlobalAnchor;
    scalar angle=0;

    StateInSlide() {
    }

    StateInSlide(AnchorPtr p) : anchor(p) {}

    StateInSlide(const vec2& x)  {
        setOffset(x);
    }

    void setOffset(const vec2& x) {
        placer = [x] (const vec2& p) {
            return p + x;
        };
    }

    vec2 getPosition() const {
        return placer(anchor->getPos());
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
