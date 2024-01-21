#include "PointCloud.h"

UPS::PointCloud::PointCloud(const vecs &P) : points(P),original_points(P)
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

UPS::PointCloud::PointCloudPtr UPS::PointCloud::applyDynamic(const VertexTimeMap &phi)
{
    PointCloudPtr rslt = NewPrimitive<PointCloud>(original_points);
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
    initPolyscopeData(pc);
}
