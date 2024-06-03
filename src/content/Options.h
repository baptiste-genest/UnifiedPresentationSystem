#ifndef OPTIONS_H
#define OPTIONS_H

#define STRING(x) #x
#define TOSTRING(x) STRING(x)
#include <string>

#include "Eigen/Dense"

namespace slope {
struct Options{

/// Global slope build prefix
static   std::string SlopePath;
static   std::string DataPath;
static   std::string ProjectName;
static   std::string ProjectPath;
static   std::string ProjectViewsPath;

/// Latex paths
static std::string Slope_PDFLATEX;
static std::string Slope_CONVERT;

///Window size
static size_t Slope_screen_resolution_x;
static size_t Slope_screen_resolution_y;

///Density for the PDF -> PNG export
static size_t Slope_density;

static Eigen::Vector3d DefaultBackgroundColor;

static bool ignore_cache;

/// Height ratio for title
static double Slope_TITLE;
static double Slope_default_height_ratio;

};
}
#endif //OPTIONS_H
