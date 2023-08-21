#include "PointCloud.h"

UPS::PointCloud::PointCloud()
{

}

UPS::PointCloud::PointCloudPtr UPS::PointCloud::AddPointCloud(const vecs &P)
{
    auto rslt = std::make_shared<PointCloud>();
    rslt->points = P;
    rslt->pc = polyscope::registerPointCloud("PC",P);
    rslt->pc->setEnabled(false);
    Primitive::addPrimitive(rslt);
    rslt->pc->setPosition(glm::vec3(1.,0,0));
    return rslt;
}

void UPS::PointCloud::draw(TimeTypeSec ts, const StateInSlide &sis)
{
    pc->setTransparency(1);

}

void UPS::PointCloud::intro(parameter t, const StateInSlide &sis)
{
    pc->setEnabled(true);
    pc->setTransparency(t);
}

void UPS::PointCloud::outro(parameter t, const StateInSlide &sis)
{
    if (t > 0.9)
        pc->setEnabled(false);
    pc->setTransparency(1-t);
}
