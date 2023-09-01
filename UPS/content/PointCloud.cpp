#include "PointCloud.h"

UPS::PointCloud::PointCloud(const vecs &P) : points(P)
{
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::Add(const vecs &P)
{
    auto rslt = NewPrimitive<PointCloud>(P);
    return rslt;
}

void UPS::PointCloud::initPolyscope()
{
    pc = polyscope::registerPointCloud("PC",points);
    initPolyscopeData(pc);
}
