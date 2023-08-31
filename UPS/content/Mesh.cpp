#include "Mesh.h"

UPS::Mesh::MeshPtr UPS::Mesh::Add(const std::string &objfile,const vec& scale)
{
    std::vector<std::array<double,3>> V;
    Faces F;
    polyscope::loadPolygonSoup(objfile,V,F);

    vecs verts(V.size());
    for (int i = 0;i<V.size();i++){
        const auto& x = V[i];
        verts[i] = vec(x[0]*scale(0),x[1]*scale(1),x[2]*scale(2));
    }

    MeshPtr rslt = NewPrimitive<Mesh>(verts,verts,F);
    return rslt;
}

UPS::Mesh::MeshPtr UPS::Mesh::apply(const mapping &phi) const
{
    auto Vphi = vertices;
    for (auto& x : Vphi)
        x = phi(x);
    MeshPtr rslt = NewPrimitive<Mesh>(Vphi,original_vertices,faces);
    return rslt;
}

UPS::Mesh::MeshPtr UPS::Mesh::applyDynamic(const time_mapping &phi) const
{
    MeshPtr rslt = NewPrimitive<Mesh>(vertices,original_vertices,faces);
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

void UPS::Mesh::initPolyscope()
{
    pc = polyscope::registerSurfaceMesh(getPolyscopeName(),vertices,faces);
    initPolyscopeData(pc);
    pc->setEdgeWidth(0.);
}


UPS::Mesh::Mesh(const vecs &vertices, const vecs &original_vertices, const Faces &faces) : vertices(vertices),
    original_vertices(original_vertices),
    faces(faces)
{
    Mesh::initPolyscope();
}
