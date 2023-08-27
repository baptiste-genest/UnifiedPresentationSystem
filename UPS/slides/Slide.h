#ifndef SLIDE_H
#define SLIDE_H

#include "../content/primitive.h"

namespace UPS {

struct Transition{
    PrimitiveID pid;
    StateInSlide state_a;
    StateInSlide state_b;
};
using Transitions = std::vector<Transition>;

struct Slide : std::map<PrimitiveID,StateInSlide> {
    void add(PrimitivePtr p,const StateInSlide& sis = {}){
        (*this)[p->pid] = sis;
    }
    void add(PrimitivePtr p,const Vec2& pos){
        (*this)[p->pid] = StateInSlide{pos};
    }

    void remove(PrimitivePtr ptr) {
        this->erase(ptr->pid);
    }

};

}

#endif // SLIDE_H
