#ifndef POINT_H
#define POINT_H

#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class Point : public PolyscopePrimitive
{
public:
    Point(const param &phi, scalar radius);

    Point() {}
    using PointPtr = std::shared_ptr<Point>;

    static PointPtr Add(const param& phi,scalar rad = 0.05);

    polyscope::PointCloud* pc;


    PointPtr apply(const mapping& f) const;
    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
private:
    vec x;
    param phi;
    scalar radius;

};

}

#endif // POINT_H
