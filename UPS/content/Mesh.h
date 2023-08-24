#ifndef MESH_H
#define MESH_H
#include "../UPS.h"
#include "PolyscopePrimitive.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/surface_mesh_io.h"
#include "../math/geometry.h"

namespace UPS {

class Mesh : public PolyscopePrimitive
{
public:
    using MeshPtr = std::shared_ptr<Mesh>;
    Mesh() {}

    static MeshPtr Add(const std::string& objfile);

    MeshPtr apply(const mapping& phi) const;

    polyscope::SurfaceMesh* pc;

private:
    vecs vertices;
    Faces faces;
};

}

#endif // MESH_H
