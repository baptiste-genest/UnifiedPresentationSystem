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
    Mesh(const vecs &vertices,const vecs& original_vertices, const Faces &faces);

    static MeshPtr Add(const std::string& objfile,const vec& scale = vec(1.,1.,1.));
    static MeshPtr Add(const std::string& objfile,scalar scale){
        return Add(objfile,vec::Ones()*scale);
    }

    MeshPtr apply(const mapping& phi) const;
    MeshPtr applyDynamic(const time_mapping& phi) const;

    using scalar_func = std::function<scalar(const vec&)>;
    using vector_func = std::function<vec(const vec&)>;

    Vec eval(const scalar_func& f) const;
    vecs eval(const vector_func& f) const;

    polyscope::SurfaceMesh* pc;

    const vecs& getVertices() const {return vertices;}

    void updateMesh(const vecs& X);

private:
    vecs vertices,original_vertices;
    Faces faces;

    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
};

}

#endif // MESH_H
