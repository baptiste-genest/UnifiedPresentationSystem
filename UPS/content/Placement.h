#ifndef PLACEMENT_H
#define PLACEMENT_H

#include "primitive.h"

namespace UPS {

const Vec2 CENTER(0.5,0.5);
const Vec2 TOP(0.5,0.1);
const Vec2 BOTTOM(0.5,0.9);


struct Placement {

    Placement(PrimitivePtr ptr) : ptr(ptr) {}

    virtual Vec2 computePlacement(const PrimitiveInSlide& other) const = 0;

    PrimitivePtr ptr;
};

struct PlaceBelow : public Placement {
    PlaceBelow(PrimitivePtr ptr,scalar padding = 0.01) : Placement(ptr),padding(padding) {}

    Vec2 computePlacement(const PrimitiveInSlide& other) const override {
        Vec2 P;
        P.x = other.second.relative_anchor_pos.x;
        P.y = other.second.relative_anchor_pos.y
              + other.first->getRelativeSize().y*0.5
              + padding
              + ptr->getRelativeSize().y*0.5
            ;
        return P;
    }

    scalar padding;
};


struct PlaceAbove : public Placement {
    PlaceAbove(PrimitivePtr ptr,scalar padding = 0.01) : Placement(ptr),padding(padding) {}

    Vec2 computePlacement(const PrimitiveInSlide& other) const override {
        Vec2 P;
        P.x = other.second.relative_anchor_pos.x;
        P.y = other.second.relative_anchor_pos.y
              - other.first->getRelativeSize().y*0.5
              - padding
              - ptr->getRelativeSize().y*0.5
            ;
        return P;
    }

    scalar padding;
};


}

#endif // PLACEMENT_H
