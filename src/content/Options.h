#ifndef OPTIONS_H
#define OPTIONS_H

#include <string>

namespace UPS {
  namespace Options{

    /// Global UPS build prefix
    extern std::string UPS_data_prefix;
    extern std::string UPS_projects_prefix;
    extern std::string PROJECT_ROOT;

    /// Latex paths
    extern std::string UPS_pathPDFLATEX;
    extern std::string UPS_pathCONVERT;
    
    ///Window size
    extern size_t UPS_screen_resolution_x;
    extern size_t UPS_screen_resolution_y;
    
    ///Density for the PDF -> PNG export
    extern size_t UPS_density;

    ///Various sizes
    ///
  
    /// Height ratio for title
    extern double UPS_TITLE;
    extern double UPS_default_height_ratio;
    
  }
}
#endif //OPTIONS_H
