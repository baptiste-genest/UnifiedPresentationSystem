#ifndef SCREENPRIMITIVE_H
#define SCREENPRIMITIVE_H

#include "primitive.h"
#include "StateInSlide.h"

namespace UPS {
class ScreenPrimitive;
using ScreenPrimitivePtr = std::shared_ptr<ScreenPrimitive>;

class ScreenPrimitive : public Primitive
{
public:
    ScreenPrimitive() {}

    static ScreenPrimitivePtr get(PrimitiveID id) {
        return std::dynamic_pointer_cast<ScreenPrimitive>(Primitive::get(id));
    }

    bool isScreenSpace() const override {
        return true;
    }

    using InstanceInSlide = std::pair<ScreenPrimitivePtr,StateInSlide>;

    inline InstanceInSlide at(const vec2& p,scalar alpha=1) {
        StateInSlide sis; sis.p = std::make_shared<AbsolutePosition>(p);
        sis.alpha = alpha;
        return {get(pid),sis};
    }

    inline InstanceInSlide at(scalar x,scalar y,scalar alpha=1) {
        return at(vec2(x,y),alpha);
    }


    inline InstanceInSlide at(std::string label) {
        StateInSlide sis;
        sis.p = std::make_shared<PersistantPosition>(label);
        return {get(pid),sis};
    }

    virtual vec2 getSize() const = 0;

    Size getRelativeSize() const {
        auto S = Vec2(Options::UPS_screen_resolution_x, UPS::Options::UPS_screen_resolution_y);
        auto s = getSize();
        return Size(s(0)/S.x,s(1)/S.y);
    }

};


}

#endif // SCREENPRIMITIVE_H
