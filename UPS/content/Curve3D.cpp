#include "Curve3D.h"

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const vecs &nodes, bool loop)
{
    Curve3DPtr rslt = NewPrimitive<Curve3D>();
    rslt->nodes = nodes;
    if (loop)
        rslt->pc = polyscope::registerCurveNetworkLoop(getPolyscopeName(),nodes);
    else
        rslt->pc = polyscope::registerCurveNetworkLine(getPolyscopeName(),nodes);
    rslt->init_polyscope_data(rslt->pc);
    return rslt;
}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const param &param,int N, bool loop)
{
    Curve3DPtr rslt = NewPrimitive<Curve3D>();
    rslt->nodes.resize(N);
    scalar dt = loop ? 1./N : 1./(N-1);

    for (int i  = 0;i<N;i++){
        auto t = i*dt;
        rslt->nodes[i] = param(t);
    }

    if (loop)
        rslt->pc = polyscope::registerCurveNetworkLoop(getPolyscopeName(),rslt->nodes);
    else
        rslt->pc = polyscope::registerCurveNetworkLine(getPolyscopeName(),rslt->nodes);
    rslt->init_polyscope_data(rslt->pc);
    return rslt;

}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::apply(const mapping &phi,bool loop) const
{
    Curve3DPtr rslt = NewPrimitive<Curve3D>();
    rslt->nodes = nodes;
    for (auto& x : rslt->nodes)
        x = phi(x);

    if (loop)
        rslt->pc = polyscope::registerCurveNetworkLoop(getPolyscopeName(),rslt->nodes);
    else
        rslt->pc = polyscope::registerCurveNetworkLine(getPolyscopeName(),rslt->nodes);
    rslt->init_polyscope_data(rslt->pc);
    return rslt;
}
