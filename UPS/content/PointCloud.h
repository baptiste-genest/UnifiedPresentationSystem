#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "PolyscopePrimitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class PointCloud : public PolyscopePrimitive
{
public:
    PointCloud(const vecs& P);

    using PointCloudPtr = std::shared_ptr<PointCloud>;

    static PointCloudPtr Add(const vecs& P);

    polyscope::PointCloud* pc;
private:
    vecs points;

    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
};

}

#endif // POINTCLOUD_H
