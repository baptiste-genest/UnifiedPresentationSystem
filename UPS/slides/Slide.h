#ifndef SLIDE_H
#define SLIDE_H

#include "SlideObject.h"
#include "../content/AssetManager.h"
#include "../content/primitive.h"

namespace UPS {

struct Transition{
    PrimitiveID pid;
    StateInSlide state_a;
    StateInSlide state_b;
};
using Transitions = std::vector<Transition>;
using PrimitiveInSlide = std::pair<PrimitiveID,StateInSlide>;

struct Slide : std::map<PrimitiveID,StateInSlide> {
    void add(PrimitivePtr p,const StateInSlide& sis){
        (*this)[p->pid] = sis;
    }
    void add(PrimitivePtr p,const Vec2& pos){
        (*this)[p->pid] = {pos,1.};
    }

};

}

#endif // SLIDE_H
