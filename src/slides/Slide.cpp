#include "Slide.h"
#include "../content/Referential.h"

namespace UPS {
vec2 RelativePosition::getPosition(const Slide& S) const {
    return placement(S.at(ref).p->getPosition(S));
}

}
