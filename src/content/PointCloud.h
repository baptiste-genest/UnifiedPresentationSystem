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
    PointCloudPtr apply(const mapping& phi);

    polyscope::PointCloud* pc;
    const vecs& getPoints() const {return points;}
private:
    vecs points;

    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
};

}

#endif // POINTCLOUD_H
