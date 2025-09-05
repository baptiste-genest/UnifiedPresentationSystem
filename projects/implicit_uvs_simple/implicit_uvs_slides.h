#pragma once
#include "../../src/slope.h"

struct ImplicitUVsSlides {
    slope::Mesh::MeshPtr disk,grid,torus,cube,grid30,grid200,sphere;

    static ImplicitUVsSlides& getContext(){
        static ImplicitUVsSlides* context;
        if (!context) context = new ImplicitUVsSlides();
        return *context;
    }
};

void CreateContextSlides(slope::Slideshow& show);
void CreateIntroSlides(slope::Slideshow& show);
void CreateLogMapSlides(slope::Slideshow& show);
void CreateExpMapSlides(slope::Slideshow& show);
void CreateSphereTracingSlides(slope::Slideshow& show);
void CreateMovingSlides(slope::Slideshow& show);
void CreateCurveBasedSlides(slope::Slideshow& show);
void CreateMultipleSeedsSlides(slope::Slideshow& show);
void CreateMergingFieldsSlides(slope::Slideshow& show);
void CreateCompactSupportSlides(slope::Slideshow& show);
void CreateAtlasComputeSlides(slope::Slideshow& show);
void CreateDetailsSlides(slope::Slideshow& show);
void CreateConclusionSlides(slope::Slideshow& show);

inline void LoadCommon() {
    using namespace slope;
    ImplicitUVsSlides::getContext().disk = slope::Mesh::Add(Options::DataPath+"meshes/disk_coarse.obj");
    ImplicitUVsSlides::getContext().grid = slope::Mesh::Add(Options::DataPath+"meshes/grid_quad_10.obj");
    ImplicitUVsSlides::getContext().grid30 = slope::Mesh::Add(Options::DataPath+"meshes/grid_quad_30.obj");
    ImplicitUVsSlides::getContext().grid200 = slope::Mesh::Add(Options::DataPath+"meshes/grid_quad_200.obj");
    ImplicitUVsSlides::getContext().torus = slope::Mesh::Add(Options::DataPath+"meshes/torus.obj");
    ImplicitUVsSlides::getContext().cube = slope::Mesh::Add(Options::DataPath+"meshes/cube.obj",1,false);
    ImplicitUVsSlides::getContext().sphere = slope::Mesh::Add(Options::DataPath+"meshes/ico_sphere_5.obj",1,true);

}


using namespace slope;
inline vec ExpSphere(const vec& x,const vec& v) {
    scalar l = v.norm();
    if (l < 1e-5)
        return x;
    return x*cos(l) + v*sin(l)/l;
}

inline vec LogSphere(const vec& x,const vec& y) {
    if ((x-y).norm() < 1e-5)
        return vec::Zero();
    return OrthoprojAagainstB(y-x,x).normalized()*std::acos(x.normalized().dot(y.normalized()));
}

inline scalar DistSphere(const vec& x,const vec& y) {
    if ((x-y).norm() < 1e-5)
        return 0;
    return std::acos(x.normalized().dot(y.normalized()));
}