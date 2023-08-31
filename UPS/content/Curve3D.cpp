#include "Curve3D.h"

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const vecs &nodes, bool loop)
{
    return NewPrimitive<Curve3D>(nodes,loop);
}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const param &param,int N, bool loop)
{
    return NewPrimitive<Curve3D>(param,N,loop);
}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::apply(const mapping &phi,bool loop) const
{
    auto X = nodes;
    for (auto& x : X)
        x = phi(x);
    return NewPrimitive<Curve3D>(X,loop);
}

void UPS::Curve3D::initPolyscope()
{
    if (loop)
        pc = polyscope::registerCurveNetworkLoop(getPolyscopeName(),nodes);
    else
        pc = polyscope::registerCurveNetworkLine(getPolyscopeName(),nodes);
    initPolyscopeData(pc);
}


namespace UPS {
Curve3D::Curve3D(const vecs &nodes,bool loop) : loop(loop),
    nodes(nodes)
{
    Curve3D::initPolyscope();
}

Curve3D::Curve3D(const param &param, int N, bool loop) : loop(loop)
{
    nodes.resize(N);
    scalar dt = loop ? 1./N : 1./(N-1);

    for (int i  = 0;i<N;i++){
        auto t = i*dt;
        nodes[i] = param(t);
    }
    Curve3D::initPolyscope();
}

}
