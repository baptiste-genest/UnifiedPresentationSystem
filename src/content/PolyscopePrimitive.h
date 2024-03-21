#ifndef POLYSCOPEPRIMITIVE_H
#define POLYSCOPEPRIMITIVE_H

#include "primitive.h"
#include "polyscope/structure.h"
#include "polyscope/surface_mesh.h"
#include "StateInSlide.h"

namespace UPS {
class PolyscopePrimitive;
using PolyscopePrimitivePtr = std::shared_ptr<PolyscopePrimitive>;

class PolyscopePrimitive : public Primitive
{
public:
    PolyscopePrimitive() {}

    void initPolyscopeData(polyscope::Structure* pcptr) {
        polyscope_ptr = pcptr;
        polyscope_ptr->setEnabled(false);
        count++;
    }

    inline std::string getPolyscopeName() const {
        return "polyscope_obj" + std::to_string(pid);
    }

    inline PrimitiveInSlide at(scalar alpha=1) {
        StateInSlide sis;
        sis.alpha = alpha;
        return {get(pid),sis};
    }

    // Primitive interface
public:
    void draw(const TimeObject&, const StateInSlide &sis) override {
        polyscope_ptr->setTransparency(sis.alpha);
    }
    void intro(const TimeObject& t,const StateInSlide &sis) override {
        polyscope_ptr->setTransparency(smoothstep(t.transitionParameter)*sis.alpha);
        updater(t(this),this);
    }
    void outro(const TimeObject& t,const StateInSlide &sis) override {
        polyscope_ptr->setTransparency(smoothstep(1-t.transitionParameter)*sis.alpha);
        updater(t(this),this);
    }



    void forceDisable() override {
        polyscope_ptr->setEnabled(false);
        //polyscope_ptr->remove();
    }

    void forceEnable() override {
        polyscope_ptr->setEnabled(true);
        polyscope_ptr->setTransparency(0);
        //initPolyscope();
    }
    bool isScreenSpace() const override {return false;}


protected:
    polyscope::Structure* polyscope_ptr;
    static size_t count;

};
template<class T>
class PolyscopeQuantity : public Primitive
{
public :
    using PCQuantityPtr = std::shared_ptr<PolyscopeQuantity>;
    static PCQuantityPtr Add(T* ptr) {
        auto rslt = NewPrimitive<PolyscopeQuantity<T>>();
        rslt->q = ptr;
        rslt->q->setEnabled(false);
        return rslt;
    }

    // Primitive interface
public:
    T* q;
    void draw(const TimeObject &time, const StateInSlide &sis) override {q->setEnabled(true);}
    void intro(const TimeObject& t, const StateInSlide &sis) override {q->setEnabled(true);}
    void outro(const TimeObject& t, const StateInSlide &sis) override {q->setEnabled(false);}
    void forceDisable() override {q->setEnabled(false);}
    bool isScreenSpace() const override {return false;}
};

template<typename T>
static PolyscopeQuantity<T>::PCQuantityPtr AddPolyscopeQuantity(T* ptr) {
    return PolyscopeQuantity<T>::Add(ptr);
}

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
    void intro(const TimeObject& t, const StateInSlide &sis) override {
        q->setVectorLengthScale(l0*smoothstep(t.transitionParameter),false);
        q->setEnabled(true);
    }
    void outro(const TimeObject& t, const StateInSlide &sis) override {
        q->setVectorLengthScale(l0*(1-smoothstep(t.transitionParameter)),false);
        if (t.transitionParameter > 0.95)
            q->setEnabled(false);
    }
    void forceDisable() override {q->setEnabled(false);}
    bool isScreenSpace() const override {return false;}
};

template<>
PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::PCQuantityPtr AddPolyscopeQuantity(polyscope::SurfaceVertexVectorQuantity* ptr) {
    return PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::Add(ptr);
}


}

#endif // POLYSCOPEPRIMITIVE_H
