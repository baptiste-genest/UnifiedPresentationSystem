#ifndef CURVE3D_H
#define CURVE3D_H

#include "PolyscopePrimitive.h"
#include "polyscope/curve_network.h"
#include "../math/Parametrization.h"

namespace slope {

class Curve3D : public PolyscopePrimitive
{
public:
    Curve3D() {}
    using Curve3DPtr = std::shared_ptr<Curve3D>;

    static Curve3DPtr Add(const vecs& nodes, bool loop = false, scalar r = -0.01);
    static Curve3DPtr Add(const curve_param& param,int N = 100,bool loop = false,scalar r = -0.01);
    static Curve3DPtr Add(const dynamic_curve_param& param,int N = 100,bool loop = false,scalar r = -0.01);
    using edge = std::array<int,2>;
    using edges = std::vector<edge>;


    polyscope::CurveNetwork* pc;


    Curve3DPtr apply(const mapping& phi,bool loop = false) const;

    void updateNodes(const vecs& X) {
        nodes = X;
        pc->updateNodePositions(nodes);
    }

    const vecs& getNodes() const {return nodes;}

    scalar radius = 0.01;
protected:
    bool loop;
    vecs nodes;

    // PolyscopePrimitive interface
public:
    Curve3D(const vecs &nodes,bool loop,scalar r);
    Curve3D(const curve_param& param,int N = 100,bool loop = false,scalar r = -0.01);
    virtual void initPolyscope() override;
};

class CurveNetwork : public Curve3D {
protected:
    edges E;

    // Primitive interface
public:
    virtual void initPolyscope() override;
    static std::shared_ptr<CurveNetwork> Add(const vecs& nodes,const edges& E,scalar r = -0.01);
    CurveNetwork(const vecs &nodes,const edges& E,scalar r);

    void updateEdges(const edges& E) {
        this->E = E;
        pc = polyscope::registerCurveNetwork(getPolyscopeName(),nodes,E);
    }

};

struct Plot {
    using Curve3DPtr = std::shared_ptr<Curve3D>;
    static Curve3DPtr Add(const scalar_function& f,scalar x0,scalar x1,const Vec2& size,const vec& anchor,int N = 100);
};

}

#endif // CURVE3D_H
