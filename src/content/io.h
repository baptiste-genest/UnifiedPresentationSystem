#ifndef IO_H
#define IO_H
#include "../UPS.h"
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/surface/meshio.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

#include <fstream>

namespace UPS {

namespace io {

inline bool file_exists(std::string filename) {
    return std::ifstream(filename).good();
}


using namespace std;
using namespace Eigen;

//https://aleksandarhaber.com/eigen-matrix-library-c-tutorial-saving-and-loading-data-in-from-a-csv-file/
void SaveMatrix(string fileName,const Mat& M);

Mat LoadMatrix(string fileToOpen);

inline bool MatrixCache(std::string file,Mat& M){
    if (file_exists(file)){
        M = LoadMatrix(file);
        return true;
    }
    return false;
}


struct GeometryCentralMesh {
    std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> position_geometry;
    GeometryCentralMesh() {
    }
    void init(std::string filename) {
        std::tie(mesh, position_geometry) = geometrycentral::surface::readManifoldSurfaceMesh(filename);
    }
    GeometryCentralMesh(std::string filename) {
        init(filename);
    }
    vec getPos(geometrycentral::surface::Vertex v) const {
        const auto& x = position_geometry->vertexPositions[v];
        return vec(x[0],x[1],x[2]);
    }
    vec getPos(int v) const {
        const auto& x = position_geometry->vertexPositions[mesh->vertex(v)];
        return vec(x[0],x[1],x[2]);
    }
};

}

}

#endif // IO_H
