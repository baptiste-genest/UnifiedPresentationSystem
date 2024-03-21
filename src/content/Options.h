#ifndef OPTIONS_H
#define OPTIONS_H

#define STRING(x) #x
#define TOSTRING(x) STRING(x)
#include <string>

namespace UPS {
struct Options{

/// Global UPS build prefix
static   std::string UPSPath;
static   std::string DataPath;
static   std::string ProjectName;
static   std::string ProjectPath;
static   std::string ProjectViewsPath;

/// Latex paths
static std::string UPS_PDFLATEX;
static std::string UPS_CONVERT;

///Window size
static size_t UPS_screen_resolution_x;
static size_t UPS_screen_resolution_y;

///Density for the PDF -> PNG export
static size_t UPS_density;

///Various sizes
///

/// Height ratio for title
static double UPS_TITLE;
static double UPS_default_height_ratio;

};
}
#endif //OPTIONS_H
