#include "PointCloud.h"

UPS::PointCloud::PointCloud()
{

}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::Add(const vecs &P)
{
    auto rslt = NewPrimitive<PointCloud>();
    rslt->points = P;
    rslt->pc = polyscope::registerPointCloud("PC",P);
    rslt->initPolyscopeData(rslt->pc);
    return rslt;
}
