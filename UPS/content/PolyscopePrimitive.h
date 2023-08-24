#ifndef POLYSCOPEPRIMITIVE_H
#define POLYSCOPEPRIMITIVE_H

#include "primitive.h"
#include "polyscope/structure.h"

namespace UPS {

class PolyscopePrimitive : public Primitive
{
public:
    PolyscopePrimitive() {}

    void init_polyscope_data(polyscope::Structure* pcptr) {
        polyscope_ptr = pcptr;
        polyscope_ptr->setEnabled(false);
        count++;
    }

    inline static std::string getPolyscopeName() {
        return "polyscope_obj" + std::to_string(count);
    }

    // Primitive interface
public:
    void draw(TimeTypeSec ts, const StateInSlide &sis) override {
        polyscope_ptr->setEnabled(true);
        polyscope_ptr->setTransparency(sis.alpha);
    }
    void intro(parameter t, const StateInSlide &sis) override {
        polyscope_ptr->setEnabled(true);
        polyscope_ptr->setTransparency(smoothstep(t)*sis.alpha);
    }
    void outro(parameter t, const StateInSlide &sis) override {
        if (t > 0.9)
            polyscope_ptr->setEnabled(false);
        polyscope_ptr->setTransparency(smoothstep(1-t)*sis.alpha);
    }

    void forceDisable() override {
        polyscope_ptr->setEnabled(false);
    }

protected:
    polyscope::Structure* polyscope_ptr;
    static size_t count;

};

}

#endif // POLYSCOPEPRIMITIVE_H
