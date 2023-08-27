#include "Point.h"

UPS::Point::PointPtr UPS::Point::Add(const param &phi,scalar r)
{
    PointPtr rslt = NewPrimitive<Point>();
    vecs X = {phi(0)};
    rslt->radius = r;
    rslt->phi = phi;
    rslt->updater = [phi](TimeTypeSec t,PrimitiveID id){
        auto p = Primitive::get<Point>(id);
        vecs X = {phi(t)};
        p->pc->updatePointPositions(X);
    };
    rslt->pc = polyscope::registerPointCloud(getPolyscopeName(),X);
    rslt->pc->setPointRadius(r,false);
    rslt->initPolyscopeData(rslt->pc);
    return rslt;
}

UPS::Point::PointPtr UPS::Point::apply(const mapping &f) const
{
    auto Phi = phi;
    return Point::Add([f,Phi](scalar t){
        return f(Phi(t));
    },radius);
}
