#ifndef PLACEMENT_H
#define PLACEMENT_H

#include "primitive.h"

namespace UPS {

const Vec2 CENTER(0.5,0.5);
const Vec2 TOP(0.5,0.1);
const Vec2 BOTTOM(0.5,0.9);


struct RelativePlacement {

    RelativePlacement(PrimitivePtr ptr) : ptr(ptr) {}

    virtual Vec2 computePlacement(const PrimitiveInSlide& other) const = 0;

    PrimitivePtr ptr;
};

inline PrimitiveInSlide PlaceABelowB(PrimitivePtr ptr,const PrimitiveInSlide& other,scalar padding = 0.01) {
    Vec2 P;
    P.x = other.second.relative_anchor_pos.x;
    P.y = other.second.relative_anchor_pos.y
            + other.first->getRelativeSize().y*0.5
            + padding
            + ptr->getRelativeSize().y*0.5
            ;
    return {ptr,P};
}

struct PlaceBelow : public RelativePlacement {
    PlaceBelow(PrimitivePtr ptr,scalar padding = 0.01) : RelativePlacement(ptr),padding(padding) {}

    Vec2 computePlacement(const PrimitiveInSlide& other) const override {
        return PlaceABelowB(ptr,other,padding).second.relative_anchor_pos;
    }

    scalar padding;
};



struct PlaceAbove : public RelativePlacement {
    PlaceAbove(PrimitivePtr ptr,scalar padding = 0.01) : RelativePlacement(ptr),padding(padding) {}

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

inline PrimitiveInSlide PlaceLeft(PrimitivePtr ptr,scalar y = 0.5,scalar padding = 0.1) {
    Vec2 P;
    P.x = ptr->getRelativeSize().x*0.5+padding;
    P.y = y;
    return {ptr,StateInSlide(P)};
}

inline PrimitiveInSlide PlaceRight(PrimitivePtr ptr,scalar y = 0.5,scalar padding = 0.01) {
    Vec2 P;
    P.x = 1-ptr->getRelativeSize().x*0.5-padding;
    P.y = y;
    return {ptr,StateInSlide(P)};
}


}

#endif // PLACEMENT_H
