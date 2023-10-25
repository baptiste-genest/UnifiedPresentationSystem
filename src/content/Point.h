#ifndef POINT_H
#define POINT_H

#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class Point : public PolyscopePrimitive
{
public:
    using VectorQuantity = PolyscopeQuantity<polyscope::PointCloudVectorQuantity>;
    using VectorQuantityPtr = VectorQuantity::PCQuantityPtr;
    Point(const param &phi, scalar radius);
    Point(const vec &x,scalar radius) : x(x),radius(radius) {
        phi = [x](scalar){return x;};
        updater = [](const TimeObject&,Primitive* ptr){
            auto p = Primitive::get<Point>(ptr->pid);
            p->updateVectors();
        };
    }

    Point() {}
    using PointPtr = std::shared_ptr<Point>;

    static PointPtr Add(const param& phi,scalar rad = 0.05);
    static PointPtr Add(const vec& x,scalar rad = 0.05);

    VectorQuantityPtr addVector(const param& phi);
    polyscope::PointCloud* pc;


    void updateVectors();
    PointPtr apply(const mapping& f) const;
    vec getCurrentPos() const {return x;}
    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
private:
    vec x;
    param phi;
    scalar radius;
    std::vector<std::pair<VectorQuantityPtr,param>> vectors;

};

}

#endif // POINT_H
