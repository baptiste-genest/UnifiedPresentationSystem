#ifndef SCREENPRIMITIVE_H
#define SCREENPRIMITIVE_H

#include "primitive.h"
#include "StateInSlide.h"
#include "Anchor.h"

namespace UPS {
class ScreenPrimitive;
using ScreenPrimitivePtr = std::shared_ptr<ScreenPrimitive>;
using ScreenPrimitiveInSlide = std::pair<ScreenPrimitivePtr,StateInSlide>;

class ScreenPrimitive : public Primitive
{
protected:
    AnchorPtr anchor;
public:
    ScreenPrimitive() {
        anchor = AbsoluteAnchor::Add(vec2(0,0));
    }

    static ScreenPrimitivePtr get(PrimitiveID id) {
        return std::dynamic_pointer_cast<ScreenPrimitive>(Primitive::get(id));
    }

    bool isScreenSpace() const override {
        return true;
    }

    AnchorPtr getAnchor() const {return anchor;}

    inline void updateAnchor(const vec2& p){
        anchor->updatePos(p);
    }


    inline ScreenPrimitiveInSlide at(const vec2& p,scalar alpha=1) {
        StateInSlide sis(p);
        anchor->updatePos(p);
        sis.alpha = alpha;
        return {get(pid),sis};
    }

    inline ScreenPrimitiveInSlide at(StateInSlide sis) {
        return {get(pid),sis};
    }


    inline ScreenPrimitiveInSlide at(scalar x,scalar y,scalar alpha=1) {
        return at(vec2(x,y),alpha);
    }


    inline ScreenPrimitiveInSlide at(std::string label,scalar alpha = 1) {
        StateInSlide sis;
        sis.anchor = LabelAnchor::Add(label);
        sis.alpha = alpha;
        return {get(pid),sis};
    }

    inline ScreenPrimitiveInSlide at(const std::function<vec2()>& placer) {
        StateInSlide sis;
        sis.anchor = DynamicAnchor::Add(placer);
        return {get(pid),sis};
    }
    inline ScreenPrimitiveInSlide track(const std::function<vec()>& toTrack,vec2 offset = vec2::Zero()) {
        StateInSlide sis;
        sis.anchor = DynamicAnchor::AddTracker(toTrack);
        offset(1) *= -1;
        sis.placer = [offset](vec2 p) { return vec2(p+offset); };
        return {get(pid),sis};
    }
    inline ScreenPrimitiveInSlide at(const vec& worldPos,const vec2& offset = vec2::Zero()) {
        StateInSlide sis;
        sis.anchor = DynamicAnchor::Add(worldPos);
        sis.placer = [offset](vec2 p) { return vec2(p+offset); };
        return {get(pid),sis};
    }


    virtual vec2 getSize() const = 0;

    Size getRelativeSize() const {
        auto s = getSize();
        return Size(s(0)/Options::UPS_screen_resolution_x,s(1)/Options::UPS_screen_resolution_y);
    }

};

struct TextualPrimitive : public ScreenPrimitive {
    std::string content;
};
using TextualPrimitivePtr = std::shared_ptr<TextualPrimitive>;


}

#endif // SCREENPRIMITIVE_H
