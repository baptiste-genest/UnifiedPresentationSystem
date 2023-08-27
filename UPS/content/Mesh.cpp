#include "Mesh.h"

UPS::Mesh::MeshPtr UPS::Mesh::Add(const std::string &objfile)
{
    MeshPtr rslt = NewPrimitive<Mesh>();
    std::vector<std::array<double,3>> V;
    polyscope::loadPolygonSoup(objfile,V,rslt->faces);

    rslt->vertices.resize(V.size());
    for (int i = 0;i<V.size();i++){
        const auto& p = V[i];
        rslt->vertices[i] = vec(p[0],p[1],p[2]);
    }

    rslt->pc = polyscope::registerSurfaceMesh(getPolyscopeName(),rslt->vertices,rslt->faces);
    rslt->initPolyscopeData(rslt->pc);
    rslt->pc->setEdgeWidth(1.);

    return rslt;
}

UPS::Mesh::MeshPtr UPS::Mesh::apply(const mapping &phi) const
{
    MeshPtr rslt = NewPrimitive<Mesh>();
    rslt->vertices = vertices;
    rslt->faces = faces;
    for (auto& x : rslt->vertices)
        x = phi(x);
    rslt->pc = polyscope::registerSurfaceMesh(getPolyscopeName(),rslt->vertices,rslt->faces);
    
    rslt->initPolyscopeData(rslt->pc);
    rslt->pc->setEdgeWidth(1.);
    return rslt;
}
