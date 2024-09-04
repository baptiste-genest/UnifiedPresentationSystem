#pragma once
#include "../../src/slope.h"

struct ImplicitUVsSlides {
    slope::Mesh::MeshPtr disk,grid,torus,cube;

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

inline void LoadCommon() {
    using namespace slope;
    ImplicitUVsSlides::getContext().disk = slope::Mesh::Add(Options::DataPath+"meshes/disk_coarse.obj");
    ImplicitUVsSlides::getContext().grid = slope::Mesh::Add(Options::DataPath+"meshes/grid_quad_10.obj");
    ImplicitUVsSlides::getContext().torus = slope::Mesh::Add(Options::DataPath+"meshes/torus.obj");
    ImplicitUVsSlides::getContext().cube = slope::Mesh::Add(Options::DataPath+"meshes/cube.obj",1,false);

}
