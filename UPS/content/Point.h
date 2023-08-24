#ifndef POINT_H
#define POINT_H

#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class Point : public PolyscopePrimitive
{
public:
    Point() {}
    using PointPtr = std::shared_ptr<Point>;

    static PointPtr Add(const param& phi,scalar rad = 0.05);

    param phi;
    polyscope::PointCloud* pc;

    PointPtr apply(const mapping& f) const;
    scalar radius;


};

}

#endif // POINT_H
