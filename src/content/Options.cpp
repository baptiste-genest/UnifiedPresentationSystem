#include "Options.h"



namespace UPS
{

std::string UPS::Options::UPSPath = "/home/eulerson314/dev/UnifiedPresentationSystem";
std::string Options::DataPath = UPSPath + "/data/";

std::string Options::ProjectName;
std::string Options::ProjectPath;
std::string Options::ProjectViewsPath;

size_t Options::UPS_screen_resolution_x = 1920;
size_t Options::UPS_screen_resolution_y = 1080;

std::string Options::UPS_PDFLATEX = "/usr/bin/pdflatex";
std::string Options::UPS_CONVERT = "/usr/bin/convert";

size_t Options::UPS_density = 800;

double Options::UPS_TITLE = 0.07;
double Options::UPS_default_height_ratio = 0.04;

}
