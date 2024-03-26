#include "PointCloud.h"

UPS::PointCloud::PointCloud(const vecs &P,scalar r) : points(P),original_points(P),radius(r)
{
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::Add(const vecs &P,scalar radius)
{
    return NewPrimitive<PointCloud>(P,radius);
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::apply(const mapping &phi)
{
    auto NP = points;
    for (auto& x : NP)
        x = phi(x);
    return Add(NP,radius);
}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::applyDynamic(const VertexTimeMap &phi)
{
    PointCloudPtr rslt = NewPrimitive<PointCloud>(original_points,radius);
    rslt->updater = [phi] (const TimeObject& t,Primitive* ptr) {
        auto M = Primitive::get<PointCloud>(ptr->pid);
        auto V = M->original_points;
        for (int i = 0;i<V.size();i++)
            V[i] = phi({V[i],i},t);
        M->updateCloud(V);
    };
    return rslt;
}

void UPS::PointCloud::initPolyscope()
{
    pc = polyscope::registerPointCloud(getPolyscopeName(),points);
    if (radius > 0)
        pc->setPointRadius(radius,false);
    initPolyscopeData(pc);
}
