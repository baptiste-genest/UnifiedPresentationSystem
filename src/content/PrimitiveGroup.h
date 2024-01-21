#ifndef PRIMITIVEGROUP_H
#define PRIMITIVEGROUP_H
#include "primitive.h"
#include "Placement.h"
#include "../slides/Slide.h"

namespace UPS {

struct PrimitiveGroup {
    Slide buffer;
    Vec2 relative_anchor = Vec2(0.5,0.5);

    ScreenPrimitivePtr last_screen_primitive_inserted;

    PrimitiveGroup& operator<<(PrimitivePtr ptr) {
        buffer.add(ptr);
        if (ptr->isScreenSpace())
            last_screen_primitive_inserted = std::static_pointer_cast<ScreenPrimitive>(ptr);
        return *this;
    }

    PrimitiveGroup& operator<<(const PrimitiveInSlide& pis) {
        buffer.add(pis.first,pis.second);
        if (pis.first->isScreenSpace())
            last_screen_primitive_inserted = std::static_pointer_cast<ScreenPrimitive>(pis.first);
        return *this;
    }

    /*
    vec2 getRelativeSize() const {
        float x_min = 100,y_min = 100,x_max = 0,y_max = 0;
        bool all_none_screen = true;
        for (auto&& [ptr,sis] : buffer.getScreenPrimitives()) {
            all_none_screen = false;
            auto pos = sis.relative_anchor_pos;
            auto size = ptr->getRelativeSize();
            x_min = std::min(x_min,pos.x-size.x*0.5f);
            y_min = std::min(y_min,pos.y-size.y*0.5f);
            x_max = std::max(x_max,pos.x+size.x*0.5f);
            y_max = std::max(y_max,pos.y+size.y*0.5f);
        }
        if (all_none_screen)
            return vec2(0,0);
        return vec2(x_max-x_min,y_max-y_min);
    }

    vec2 getRelativeAnchorPos() const {
        vec2 anchor(0,0);
        int nb_inscreen = 0;
        for (auto&& [ptr,sis] : buffer.getScreenPrimitives()) {
            nb_inscreen++;
            auto pos = sis.relative_anchor_pos;
            anchor(0) += pos.x;
            anchor(1) += pos.y;
        }
        if (nb_inscreen == 0)
            return vec2(0,0);
        anchor /= nb_inscreen;
        return anchor;
    }

    inline PrimitiveGroup& operator<<(const RelativePlacement& P) {
        if (!P.ptr_other){
            auto pos = P.computePlacement(last_screen_primitive_inserted);
            buffer.add(P.ptr,pos);
        }
        else {
            auto pos = P.computePlacement({P.ptr_other,buffer[P.ptr_other->pid]});
            buffer.add(P.ptr,pos);
        }
        return *this;
    }
*/



};

}

#endif // PRIMITIVEGROUP_H
