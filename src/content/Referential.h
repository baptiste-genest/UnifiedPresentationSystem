#ifndef REFERENTIAL_H
#define REFERENTIAL_H

#include "../UPS.h"

namespace UPS {

struct Referential {
    Referential* ref = nullptr;
    vec2 offset = vec2(0,0);
    virtual vec2 getPosition() const {
        if (ref == nullptr)
            return offset;
        return ref->getPosition() + offset;
    }
};


}

#endif // REFERENTIAL_H
