#include "Point.h"

UPS::Point::PointPtr UPS::Point::Add(const curve_param &phi,scalar r)
{
    return NewPrimitive<Point>(phi,r);
}

UPS::Point::PointPtr UPS::Point::Add(const vec &x, scalar rad)
{
    return NewPrimitive<Point>(x,rad);
}

UPS::Point::VectorQuantity::PCQuantityPtr UPS::Point::addVector(const curve_param &phiX)
{
    vecs X = {phiX(0)};
    auto name = getPolyscopeName() + std::to_string(vectors.size());
    auto V = VectorQuantity::Add(pc->addVectorQuantity(name,X));
    vectors.push_back({V,phiX});
    V->q->setVectorLengthScale(X[0].norm(),false);
    V->q->setVectorRadius(0.02,false);
    return V;
}

void UPS::Point::updateVectors()
{
    for (int i = 0;i<vectors.size();i++){
        if (vectors[i].first->q->isEnabled()){
            vecs X = {vectors[i].second(getInnerTime())};
            //std::cout << "norm " << X[0].transpose() << std::endl;
            auto name = getPolyscopeName() + std::to_string(i);
            vectors[i].first->q = pc->addVectorQuantity(name,X);
            vectors[i].first->q->setVectorLengthScale(X[0].norm(),false);
            vectors[i].first->q->setVectorRadius(0.02,false);
        }
    }
}

UPS::Point::PointPtr UPS::Point::apply(const mapping &f) const
{
    auto Phi = phi;
    return Point::Add(curve_param([f,Phi](scalar t){
        return f(Phi(t));
                      }),radius);
}

void UPS::Point::initPolyscope()
{
    vecs X = {x};
    pc = polyscope::registerPointCloud(getPolyscopeName(),X);
    pc->setPointRadius(radius,false);
    initPolyscopeData(pc);
}


UPS::Point::Point(const curve_param &phi, scalar radius) : x(phi(0)),
    phi(phi),
    radius(radius)
{
    updater = [phi](const TimeObject& t,Primitive* ptr){
        auto p = Primitive::get<Point>(ptr->pid);
        p->x = phi(t.inner_time);
        vecs X = {p->x};
        p->pc->updatePointPositions(X);
        p->updateVectors();
    };
}
