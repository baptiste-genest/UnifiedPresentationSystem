#include "Point.h"

slope::Point::PointPtr slope::Point::Add(const curve_param &phi,scalar r)
{
    return NewPrimitive<Point>([phi](TimeObject t){return phi(t.inner_time);},r);
}

slope::Point::PointPtr slope::Point::Add(const vec &x, scalar rad)
{
    return NewPrimitive<Point>(x,rad);
}

slope::Point::PointPtr slope::Point::Add(const DynamicParam &phi, scalar rad)
{
    return NewPrimitive<Point>(phi,rad);
}

slope::Point::VectorQuantityPtr slope::Point::addVector(const vec &v)
{
    return addVector([v](TimeObject){return v;});
}

slope::Point::VectorQuantity::PCQuantityPtr slope::Point::addVector(const curve_param &phiX)
{
    vecs X = {phiX(0)};
    auto name = getPolyscopeName() + std::to_string(vectors.size());
    auto V = VectorQuantity::Add(pc->addVectorQuantity(name,X));
    auto phi = [phiX](TimeObject t){return phiX(t.inner_time);};
    vectors.push_back({V,phi});
    V->q->setVectorLengthScale(X[0].norm(),false);
    V->q->setVectorRadius(0.02,false);
    return V;
}

slope::Point::VectorQuantity::PCQuantityPtr slope::Point::addVector(const DynamicParam &phi)
{
    vecs X = {phi(TimeObject())};
    auto name = getPolyscopeName() + std::to_string(vectors.size());
    auto V = VectorQuantity::Add(pc->addVectorQuantity(name,X));
    vectors.push_back({V,phi});
    V->q->setVectorLengthScale(X[0].norm(),false);
    V->q->setVectorRadius(0.02,false);
    return V;
}


void slope::Point::setPos(const vec &v)
{
    x = v;
    vecs X = {v};
    pc->updatePointPositions(X);
}

void slope::Point::updateVectors(const TimeObject& t)
{
    for (int i = 0;i<vectors.size();i++){
        if (vectors[i].first->q->isEnabled()){
            vecs X = {vectors[i].second(t)};
            //std::cout << "norm " << X[0].transpose() << std::endl;
            auto name = getPolyscopeName() + std::to_string(i);
            vectors[i].first->q = pc->addVectorQuantity(name,X);
            vectors[i].first->q->setVectorLengthScale(X[0].norm(),false);
            vectors[i].first->q->setVectorRadius(0.02,false);
        }
    }
}

slope::Point::PointPtr slope::Point::apply(const mapping &f) const
{
    auto Phi = phi;
    return Point::Add(DynamicParam([f,Phi](TimeObject t){
        return f(Phi(t));
                      }),radius);
}

void slope::Point::initPolyscope()
{
    vecs X = {x};
    pc = polyscope::registerPointCloud(getPolyscopeName(),X);
    pc->setPointRadius(radius,false);
    initPolyscopeData(pc);
    pc->setPointColor(getColor());
}


slope::Point::Point(const DynamicParam &phi, scalar radius) :
    phi(phi),
    radius(radius)
{
    updater = [phi](const TimeObject& t,Primitive* ptr){
        auto p = Primitive::get<Point>(ptr->pid);
        p->setPos(phi(t));
        p->updateVectors(t);
    };
}
