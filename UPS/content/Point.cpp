#include "Point.h"

UPS::Point::PointPtr UPS::Point::Add(const param &phi,scalar r)
{
    PointPtr rslt = NewPrimitive<Point>(phi,r);
    return rslt;
}

UPS::Point::PointPtr UPS::Point::apply(const mapping &f) const
{
    auto Phi = phi;
    return Point::Add([f,Phi](scalar t){
        return f(Phi(t));
    },radius);
}

void UPS::Point::initPolyscope()
{
    vecs X = {x};
    pc = polyscope::registerPointCloud(getPolyscopeName(),X);
    pc->setPointRadius(radius,false);
    initPolyscopeData(pc);
}


UPS::Point::Point(const param &phi, scalar radius) : x(phi(0)),
    phi(phi),
    radius(radius)
{
    initPolyscope();
    updater = [phi](TimeTypeSec t,PrimitiveID id){
        auto p = Primitive::get<Point>(id);
        vecs X = {phi(t)};
        p->pc->updatePointPositions(X);
    };
}
