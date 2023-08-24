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

    polyscope::CurveNetwork* pc;

    vecs nodes;

    Curve3DPtr apply(const mapping& phi,bool loop = false) const;

};

}

#endif // CURVE3D_H
