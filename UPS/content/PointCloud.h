#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "primitive.h"
#include "polyscope/point_cloud.h"

namespace UPS {

class PointCloud : public Primitive
{
public:
    PointCloud();

    using PointCloudPtr = std::shared_ptr<PointCloud>;

    static PointCloudPtr AddPointCloud(const vecs& P);

private:
    polyscope::PointCloud* pc;
    vecs points;

    // Primitive interface
public:
    void draw(TimeTypeSec ts, const StateInSlide &sis) override;
    void intro(parameter t, const StateInSlide &sis) override;
    void outro(parameter t, const StateInSlide &sis) override;
};

}

#endif // POINTCLOUD_H
