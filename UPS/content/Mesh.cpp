#include "Mesh.h"

UPS::Mesh::MeshPtr UPS::Mesh::Add(const std::string &objfile,const vec& scale)
{
    MeshPtr rslt = NewPrimitive<Mesh>();
    std::vector<std::array<double,3>> V;
    polyscope::loadPolygonSoup(objfile,V,rslt->faces);

    rslt->vertices.resize(V.size());
    for (int i = 0;i<V.size();i++){
        const auto& p = V[i];
        rslt->vertices[i] = vec(p[0]*scale(0),p[1]*scale(1),p[2]*scale(2));
    }

    rslt->pc = polyscope::registerSurfaceMesh(getPolyscopeName(),rslt->vertices,rslt->faces);
    rslt->initPolyscopeData(rslt->pc);
    rslt->pc->setEdgeWidth(0.);
    rslt->original_vertices = rslt->vertices;

    return rslt;
}

UPS::Mesh::MeshPtr UPS::Mesh::apply(const mapping &phi) const
{
    MeshPtr rslt = NewPrimitive<Mesh>();
    rslt->vertices = vertices;
    rslt->faces = faces;
    rslt->original_vertices = vertices;
    for (auto& x : rslt->vertices)
        x = phi(x);
    rslt->pc = polyscope::registerSurfaceMesh(getPolyscopeName(),rslt->vertices,rslt->faces);
    rslt->initPolyscopeData(rslt->pc);
    rslt->pc->setEdgeWidth(0.);
    return rslt;
}

UPS::Mesh::MeshPtr UPS::Mesh::applyDynamic(const time_mapping &phi) const
{
    MeshPtr rslt = NewPrimitive<Mesh>();
    rslt->vertices = vertices;
    rslt->faces = faces;
    rslt->original_vertices = vertices;
    rslt->pc = polyscope::registerSurfaceMesh(getPolyscopeName(),rslt->vertices,rslt->faces);
    rslt->initPolyscopeData(rslt->pc);
    rslt->pc->setEdgeWidth(0.);
    rslt->updater = [phi] (TimeTypeSec t,PrimitiveID id) {
        auto M = Primitive::get<Mesh>(id);
        auto V = M->original_vertices;
        for (auto& x : V)
            x = phi(x,t);
        M->updateMesh(V);
    };
    return rslt;

}

UPS::Vec UPS::Mesh::eval(const scalar_func &f) const
{
    Vec X(vertices.size());
    for (int i = 0;i<vertices.size();i++)
        X[i] = f(vertices[i]);
    return X;
}

UPS::vecs UPS::Mesh::eval(const vector_func &f) const
{
    vecs X(vertices.size());
    for (int i = 0;i<vertices.size();i++)
        X[i] = f(vertices[i]);
    return X;
}

void UPS::Mesh::updateMesh(const vecs &X)
{
    vertices = X;
    pc->updateVertexPositions(vertices);
}
