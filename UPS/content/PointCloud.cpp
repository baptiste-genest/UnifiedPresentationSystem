#include "PointCloud.h"

UPS::PointCloud::PointCloud(const vecs &P) : points(P)
{
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::Add(const vecs &P)
{
    return NewPrimitive<PointCloud>(P);
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::apply(const mapping &phi)
{
    auto NP = points;
    for (auto& x : NP)
        x = phi(x);
    return Add(NP);
}

void UPS::PointCloud::initPolyscope()
{
    pc = polyscope::registerPointCloud(getPolyscopeName(),points);
    initPolyscopeData(pc);
}
