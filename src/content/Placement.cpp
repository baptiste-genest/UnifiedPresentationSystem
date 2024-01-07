#include "Placement.h"


UPS::Vec2 UPS::PlaceRelative::computePlacement(const PrimitiveInSlide &other) const {
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
