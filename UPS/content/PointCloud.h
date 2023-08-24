#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class PointCloud : public PolyscopePrimitive
{
public:
    PointCloud();

    using PointCloudPtr = std::shared_ptr<PointCloud>;

    static PointCloudPtr Add(const vecs& P);

private:
    polyscope::PointCloud* pc;
    vecs points;
};

}

#endif // POINTCLOUD_H
