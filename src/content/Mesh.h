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

    struct Vertex {
        vec pos;
        int id;
    };

    using VertexTimeMap = std::function<vec(const Vertex&,const TimeObject&)>;
    using VertexMap = std::function<vec(const Vertex&)>;

    using MeshPtr = std::shared_ptr<Mesh>;
    Mesh() {}
    Mesh(const vecs &vertices,const vecs& original_vertices, const Faces &faces,bool smooth = false);

    static MeshPtr Add(const std::string& objfile,const vec& scale = vec(1.,1.,1.),bool smooth = false);
    static MeshPtr Add(const std::string& objfile,scalar scale,bool smooth = false){
        return Add(objfile,vec::Ones()*scale,smooth);
    }

    static MeshPtr Add(const vecs& V,const Faces& F,bool smooth = false){
        return NewPrimitive<Mesh>(V,V,F,smooth);
    }

    MeshPtr apply(const VertexMap& phi,bool smooth = true) const;
    MeshPtr apply(const mapping& phi,bool smooth = true) const {
        return apply([phi](Vertex v){return phi(v.pos);},smooth);
    }
    MeshPtr applyDynamic(const VertexTimeMap& phi,bool smooth = true) const;

    void setSmooth(bool set);

    using scalar_func = std::function<scalar(const vec&)>;
    using vector_func = std::function<vec(const vec&)>;

    Vec eval(const scalar_func& f) const;
    vecs eval(const vector_func& f) const;

    polyscope::SurfaceMesh* pc;

    const vecs& getVertices() const {return vertices;}
    const Faces& getFaces() const {return faces;}

    void updateMesh(const vecs& X);


private:
    vecs vertices,original_vertices;
    bool smooth = false;
    Faces faces;

    // PolyscopePrimitive interface
public:
    virtual void initPolyscope() override;
};

}

#endif // MESH_H
