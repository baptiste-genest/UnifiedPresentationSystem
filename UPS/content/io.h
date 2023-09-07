#ifndef IO_H
#define IO_H
#include "../UPS.h"

#include <fstream>

namespace UPS {

namespace io {

inline bool file_exists(std::string filename) {
    return std::ifstream(filename).good();
}

#include <iostream>
#include<Eigen/Dense>
#include<fstream>

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

}

}

#endif // IO_H
