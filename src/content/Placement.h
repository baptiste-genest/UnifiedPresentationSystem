#ifndef PLACEMENT_H
#define PLACEMENT_H

#include "primitive.h"

namespace UPS {

const Vec2 CENTER(0.5,0.5);
const Vec2 TOP(0.5,0.1);
const Vec2 BOTTOM(0.5,0.9);

struct Replace {
    Replace(PrimitivePtr ptr,PrimitivePtr ptr_other = nullptr) : ptr(ptr),ptr_other(ptr_other) {}
    PrimitivePtr ptr;
    PrimitivePtr ptr_other = nullptr;
};


struct RelativePlacement {

    RelativePlacement(PrimitivePtr ptr,PrimitivePtr ptr_other = nullptr) : ptr(ptr),ptr_other(ptr_other) {}

    virtual Vec2 computePlacement(const PrimitiveInSlide& other) const = 0;

    PrimitivePtr ptr;
    PrimitivePtr ptr_other = nullptr;
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

inline PrimitiveInSlide PlaceANextToB(PrimitivePtr ptr,const PrimitiveInSlide& other,int side = 1,scalar padding = 0.01) {
    Vec2 P;
    P.y = other.second.relative_anchor_pos.y;
    P.x = other.second.relative_anchor_pos.x
          + side*(other.first->getRelativeSize().x*0.5
                    + padding
                    + ptr->getRelativeSize().x*0.5)
        ;
    return {ptr,P};
}


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
    PlaceRelative(PrimitivePtr ptr,PrimitivePtr ptr_other,placeX X,placeY Y,scalar paddingx = 0.01,scalar paddingy = 0.01) : RelativePlacement(ptr,ptr_other),X(X),Y(Y),paddingx(paddingx),paddingy(paddingy) {}

    Vec2 computePlacement(const PrimitiveInSlide& other) const override;

    scalar paddingx,paddingy;
    placeX X;
    placeY Y;
};

inline PlaceRelative PlaceNextTo(PrimitivePtr ptr,int side,scalar paddingx = 0.01,PrimitivePtr other = nullptr) {
    if (side == 1)
        return PlaceRelative(ptr,placeX::REL_RIGHT,SAME_Y,paddingx);
    return PlaceRelative(ptr,placeX::REL_LEFT,SAME_Y,paddingx);
}

inline PlaceRelative PlaceBelow(PrimitivePtr ptr,scalar paddingy = 0.01) {
    return PlaceRelative(ptr,placeX::SAME_X,placeY::REL_BOTTOM,0.01,paddingy);
}
inline PlaceRelative PlaceBelow(PrimitivePtr ptr,PrimitivePtr other,scalar paddingy = 0.01) {
    return PlaceRelative(ptr,other,placeX::SAME_X,placeY::REL_BOTTOM,0.01,paddingy);
}
inline PlaceRelative PlaceAbove(PrimitivePtr ptr,scalar paddingy = 0.01) {
    return PlaceRelative(ptr,placeX::SAME_X,placeY::REL_TOP,0.01,paddingy);
}
inline PlaceRelative PlaceAbove(PrimitivePtr ptr,PrimitivePtr other,scalar paddingy = 0.01) {
    return PlaceRelative(ptr,other,placeX::SAME_X,placeY::REL_TOP,0.01,paddingy);
}


inline PrimitiveInSlide PlaceLeft(PrimitivePtr ptr,scalar y = 0.5,scalar padding = 0.1) {
    Vec2 P;
    P.x = ptr->getRelativeSize().x*0.5+padding;
    P.y = y;
    return {ptr,StateInSlide(P)};
}


inline PrimitiveInSlide PlaceRight(PrimitivePtr ptr,scalar y = 0.5,scalar padding = 0.1) {
    Vec2 P;
    P.x = 1-ptr->getRelativeSize().x*0.5-padding;
    P.y = y;
    return {ptr,StateInSlide(P)};
}

inline PrimitiveInSlide PlaceBottom(PrimitivePtr ptr,scalar x = 0.5,scalar padding = 0.1) {
    Vec2 P;
    P.x = x;
    P.y = 1. - ptr->getRelativeSize().y*0.5+padding;
    return {ptr,StateInSlide(P)};
}

inline PrimitiveInSlide PlaceTop(PrimitivePtr ptr,scalar x = 0.5,scalar padding = 0.1) {
    Vec2 P;
    P.x = x;
    P.y = ptr->getRelativeSize().y*0.5+padding;
    return {ptr,StateInSlide(P)};
}




}

#endif // PLACEMENT_H
