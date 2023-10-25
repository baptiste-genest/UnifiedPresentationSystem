#ifndef CURVE3D_H
#define CURVE3D_H

#include "PolyscopePrimitive.h"
#include "polyscope/curve_network.h"

namespace UPS {

class Curve3D : public PolyscopePrimitive
{
public:
    Curve3D() {}
    using Curve3DPtr = std::shared_ptr<Curve3D>;

    static Curve3DPtr Add(const vecs& nodes, bool loop = false);
    static Curve3DPtr Add(const param& param,int N = 100,bool loop = false);

    void setRadius(scalar r);

    polyscope::CurveNetwork* pc;


    Curve3DPtr apply(const mapping& phi,bool loop = false) const;

    scalar radius = 0.01;
private:
    bool loop;
    vecs nodes;

    // PolyscopePrimitive interface
public:
    Curve3D(const vecs &nodes,bool loop);
    Curve3D(const param& param,int N = 100,bool loop = false);
    virtual void initPolyscope() override;
};

struct Plot {
    using Curve3DPtr = std::shared_ptr<Curve3D>;
    static Curve3DPtr Add(const scalar_function& f,scalar x0,scalar x1,const Vec2& size,const vec& anchor,int N = 100);
};

}

#endif // CURVE3D_H
