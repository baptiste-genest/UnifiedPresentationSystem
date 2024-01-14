#ifndef VECTORFIELD_H
#define VECTORFIELD_H
#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"
#include "../math/utils.h"

namespace UPS {

class VectorField : public Primitive
{
private:
    vecs V,X;
    scalar length = 1;
    polyscope::PointCloud* pc;
    polyscope::PointCloudVectorQuantity* pq;

    // Primitive interface
public:
    VectorField(const vecs &X, const vecs &V,double l0 = 0.1);
    virtual void draw(const TimeObject &time, const StateInSlide &sis) override {
        pc->setEnabled(true);
        pq->setEnabled(true);
        pq->setVectorLengthScale(length,false);
    }
    virtual void intro(const TimeObject &t, const StateInSlide &sis) override {
        pq->setVectorLengthScale(t.transitionParameter*length,false);
    }
    virtual void outro(const TimeObject &t, const StateInSlide &sis) override {
        pq->setVectorLengthScale((1-t.transitionParameter)*length,false);
    }
    virtual void forceDisable() override {
        pc->setEnabled(false);
        pq->setEnabled(false);
    }
    virtual void forceEnable() override {
        pc->setEnabled(true);
        pq->setEnabled(true);
    }

    virtual bool isScreenSpace() override {
        return false;
    }


    using VectorFieldPtr = std::shared_ptr<VectorField>;
    inline static VectorFieldPtr Add(const vecs& X,const vecs& V,scalar l) {
        return NewPrimitive<VectorField>(X,V,l);
    }
    static VectorFieldPtr AddOnGrid(const vecs& V);
};

}

#endif // VECTORFIELD_H
