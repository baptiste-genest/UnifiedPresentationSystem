#include "Curve3D.h"

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const vecs &nodes, bool loop)
{
    return NewPrimitive<Curve3D>(nodes,loop);
}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::Add(const curve_param &param,int N, bool loop)
{
    return NewPrimitive<Curve3D>(param,N,loop);
}

void UPS::Curve3D::setRadius(scalar r)
{
    radius = r;
    pc->setRadius(r,true);
}

UPS::Curve3D::Curve3DPtr UPS::Curve3D::apply(const mapping &phi,bool loop) const
{
    auto X = nodes;
    for (auto& x : X)
        x = phi(x);
    auto C = NewPrimitive<Curve3D>(X,loop);
    C->setRadius(radius);
    return C;
}

void UPS::Curve3D::initPolyscope()
{
    if (loop)
        pc = polyscope::registerCurveNetworkLoop(getPolyscopeName(),nodes);
    else
        pc = polyscope::registerCurveNetworkLine(getPolyscopeName(),nodes);
    pc->setRadius(radius,true);
    initPolyscopeData(pc);
}


namespace UPS {
Curve3D::Curve3D(const vecs &nodes,bool loop) : loop(loop),
    nodes(nodes)
{
}

Curve3D::Curve3D(const curve_param &param, int N, bool loop) : loop(loop)
{
    nodes.resize(N);
    scalar dt = loop ? 1./N : 1./(N-1);

    for (int i  = 0;i<N;i++){
        auto t = i*dt;
        nodes[i] = param(t);
    }
}

Plot::Curve3DPtr Plot::Add(const scalar_function &f, scalar x0, scalar x1,const Vec2& size,const vec& anchor, int N)
{
    scalars F(N);
    for (int i = 0;i<N;i++)
        F[i] = f(x0 + i*(x1-x0)/(N-1));
    auto min = *std::min_element(F.begin(),F.end());
    auto max = *std::max_element(F.begin(),F.end());
    std::cout << min << " " << max << std::endl;
    scalar range = max-min;
    vecs V(N);
    for (int i = 0;i<N;i++){
        V[i] = anchor + vec(i/scalar(N-1.),F[i]/range,0.);
    }
    return Curve3D::Add(V);
}

}
