#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "../UPS.h"
#include "../math/kernels.h"

namespace UPS {

struct StateInSlide {
    Vec2 relative_anchor_pos;
    scalar alpha = 1;

    Vec2 getAbsoluteAnchorPos() const {
        auto S = ImGui::GetWindowSize();
        return Vec2(relative_anchor_pos.x*S.x,relative_anchor_pos.y*S.y);
    }
};


struct Primitive {

    PrimitiveID pid;
    static std::vector<PrimitivePtr> primitives;

    static void addPrimitive(PrimitivePtr ptr) {
        ptr->pid = primitives.size();
        primitives.push_back(ptr);
    }

    static PrimitivePtr get(PrimitiveID id){
        return primitives[id];
    }

    virtual void draw(TimeTypeSec ts,const StateInSlide& sis) = 0;
    virtual void intro(parameter t,const StateInSlide& sis) = 0;
    virtual void outro(parameter t,const StateInSlide& sis) = 0;
    virtual void transition(parameter t,TimeTypeSec ts,const StateInSlide& sa,const StateInSlide& sb){
        StateInSlide St;
        St.relative_anchor_pos.x = std::lerp(sa.relative_anchor_pos.x,sb.relative_anchor_pos.x,smoothstep(t));
        St.relative_anchor_pos.y = std::lerp(sa.relative_anchor_pos.y,sb.relative_anchor_pos.y,smoothstep(t));
        St.alpha = std::lerp(sa.alpha,sb.alpha,t);
        draw(ts,St);
    }
};

}

#endif // PRIMITIVE_H
