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

struct Slide : public std::map<PrimitiveID,StateInSlide> {
    void add(PrimitivePtr p,const StateInSlide& sis = {}){
        if (p->isExclusive()){
            if (exclusive_prim != -1)
                this->erase(exclusive_prim);
            exclusive_prim = p->pid;
        }
        (*this)[p->pid] = sis;
    }
    void add(PrimitivePtr p,const Vec2& pos){
        add(p,StateInSlide{pos});
    }

    void remove(PrimitivePtr ptr) {
        this->erase(ptr->pid);
    }

    PrimitiveID exclusive_prim = -1;

};

}

#endif // SLIDE_H
