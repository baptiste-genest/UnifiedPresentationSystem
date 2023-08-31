#ifndef POLYSCOPEPRIMITIVE_H
#define POLYSCOPEPRIMITIVE_H

#include "primitive.h"
#include "polyscope/structure.h"
#include "polyscope/surface_mesh.h"

namespace UPS {

template<class T>
class PolyscopeQuantity : public Primitive
{
public :
    using PCQuantity = std::shared_ptr<PolyscopeQuantity>;
    static PCQuantity Add(T* ptr) {
        auto rslt = NewPrimitive<PolyscopeQuantity<T>>();
        rslt->q = ptr;
        rslt->q->setEnabled(false);
        return rslt;
    }

    // Primitive interface
public:
    T* q;
    void draw(const TimeObject &time, const StateInSlide &sis) override {q->setEnabled(true);}
    void intro(parameter t, const StateInSlide &sis) override {q->setEnabled(true);}
    void outro(parameter t, const StateInSlide &sis) override {q->setEnabled(false);}
    void forceDisable() override {q->setEnabled(false);}
};

template<>
class PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity> : public Primitive
{
public :
    using T = polyscope::SurfaceVertexVectorQuantity;
    using PCQuantityPtr = std::shared_ptr<PolyscopeQuantity<T>>;
    static PCQuantityPtr Add(T* ptr) {
        auto rslt = NewPrimitive<PolyscopeQuantity<T>>();
        rslt->q = ptr;
        rslt->q->setEnabled(false);
        rslt->l0 = ptr->getVectorLengthScale();
               return rslt;
    }

    // Primitive interface
public:
    scalar l0;
    T* q;
    void draw(const TimeObject &time, const StateInSlide &sis) override {
        //std::cout << "ok" << std::endl;
        q->setEnabled(true);
        q->setVectorLengthScale(l0,false);
    }
    void intro(parameter t, const StateInSlide &sis) override {
        q->setVectorLengthScale(l0*smoothstep(t),false);
        q->setEnabled(true);
    }
    void outro(parameter t, const StateInSlide &sis) override {
        q->setVectorLengthScale(l0*(1-smoothstep(t)),false);
        if (t > 0.95)
            q->setEnabled(false);
    }
    void forceDisable() override {q->setEnabled(false);}
};


class PolyscopePrimitive : public Primitive
{
public:
    PolyscopePrimitive() {}

    void initPolyscopeData(polyscope::Structure* pcptr) {
        polyscope_ptr = pcptr;
        polyscope_ptr->setEnabled(false);
        count++;
    }

    inline static std::string getPolyscopeName() {
        return "polyscope_obj" + std::to_string(count);
    }

    // Primitive interface
public:
    void draw(const TimeObject&, const StateInSlide &sis) override {
        polyscope_ptr->setEnabled(true);
        polyscope_ptr->setTransparency(sis.alpha);
    }
    void intro(parameter t,const StateInSlide &sis) override {
        polyscope_ptr->setEnabled(true);
        polyscope_ptr->setTransparency(smoothstep(t)*sis.alpha);
        updater(getInnerTime(),pid);
    }
    void outro(parameter t,const StateInSlide &sis) override {
        if (t > 0.9)
            polyscope_ptr->setEnabled(false);
        polyscope_ptr->setTransparency(smoothstep(1-t)*sis.alpha);
        updater(getInnerTime(),pid);
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
