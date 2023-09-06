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
    enum placeX {
        REL_LEFT,
        ABS_LEFT,
        CENTER_X,
        SAME_X,
        REL_RIGHT,
        ABS_RIGHT
    };
    enum placeY {
        REL_TOP,
        ABS_TOP,
        CENTER_Y,
        SAME_Y,
        REL_BOTTOM,
        ABS_BOTTOM
    };

struct PlaceRelative : public RelativePlacement {
    PlaceRelative(PrimitivePtr ptr,placeX X,placeY Y,scalar paddingx = 0.01,scalar paddingy = 0.01) : RelativePlacement(ptr),X(X),Y(Y),paddingx(paddingx),paddingy(paddingy) {}

    Vec2 computePlacement(const PrimitiveInSlide& other) const override {
        Vec2 P;
        switch(X) {
        case REL_LEFT:
                P.x = other.second.relative_anchor_pos.x
                        - other.first->getRelativeSize().x*0.5
                        - paddingx
                        - ptr->getRelativeSize().x*0.5
                        ;
                break;
            case ABS_LEFT:
                P.x = paddingx + ptr->getRelativeSize().x*0.5;
                break;
            case CENTER_X:
                P.x = CENTER.x;
                break;
            case SAME_X:
                P.x = other.second.relative_anchor_pos.x;
                break;
            case REL_RIGHT:
                P.x = other.second.relative_anchor_pos.x
                        + other.first->getRelativeSize().x*0.5
                        + paddingx
                        + ptr->getRelativeSize().x*0.5
                        ;
                break;
            case ABS_RIGHT:
                P.x = 1-paddingx-ptr->getRelativeSize().x*0.5;
                break;
        }
        switch(Y) {
            case REL_BOTTOM:
                P.y = other.second.relative_anchor_pos.y
                        + other.first->getRelativeSize().y*0.5
                        + paddingy
                        + ptr->getRelativeSize().y*0.5
                        ;
                break;
            case ABS_TOP:
                P.y = paddingy-ptr->getRelativeSize().y*0.5;
                break;
            case CENTER_Y:
                P.y = CENTER.y;
                break;
            case SAME_Y:
                P.y = other.second.relative_anchor_pos.y;
                break;
            case REL_TOP:
                P.y = other.second.relative_anchor_pos.y
                        - other.first->getRelativeSize().y*0.5
                        - paddingy
                        - ptr->getRelativeSize().y*0.5
                        ;
                break;
            case ABS_BOTTOM:
                P.y = 1-paddingy - ptr->getRelativeSize().y*0.5;
                break;
        }
        return P;
    }

    scalar paddingx,paddingy;
    placeX X;
    placeY Y;
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
