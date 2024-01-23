#ifndef SLIDE_H
#define SLIDE_H

#include "../content/primitive.h"
#include "../content/ScreenPrimitive.h"

namespace UPS {

struct Transition{
    PrimitiveID pid;
    StateInSlide state_a;
    StateInSlide state_b;
};
using Transitions = std::vector<Transition>;

struct Slide : public std::map<PrimitivePtr,StateInSlide> {

    void add(PrimitivePtr p,const StateInSlide& sis = {}){
        if (p->isExclusive()){
            if (exclusive_prim != nullptr)
                this->erase(exclusive_prim);
            exclusive_prim = p;
        }
        (*this)[p] = sis;
    }
    void add(PrimitivePtr p,const vec2& pos){
        add(p,StateInSlide(pos));
    }

    void add(PrimitiveInSlide pis){
        add(pis.first,pis.second);
    }

    void remove(PrimitivePtr ptr) {
        this->erase(ptr);
    }

    PrimitivePtr exclusive_prim = nullptr;

    std::map<ScreenPrimitivePtr,StateInSlide> getScreenPrimitives() const {
        std::map<ScreenPrimitivePtr,StateInSlide> rslt;
        for (auto&& [ptr,sis] : *this) {
            if (!ptr->isScreenSpace())
                continue;
            rslt[std::dynamic_pointer_cast<ScreenPrimitive>(ptr)] = sis;
        }
        return rslt;
    }

};

}

#endif // SLIDE_H
