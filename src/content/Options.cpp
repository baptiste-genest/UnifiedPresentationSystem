#include "Options.h"



namespace UPS
{

std::string UPS::Options::UPSPath = std::string(TOSTRING(UPS_SOURCE));
std::string Options::DataPath = UPSPath + "/data/";

std::string Options::ProjectName;
std::string Options::ProjectPath;
std::string Options::ProjectViewsPath;

std::string Options::UPS_pathPDFTEX = "";
std::string Options::UPS_pathGS = "";
std::string Options::UPS_pathPDFCROP = "";
std::string Options::UPS_pathCONVERT = "";
std::string Options::UPS_pathPDFLATEX = "";
size_t Options::UPS_screen_resolution_x = 1920;
size_t Options::UPS_screen_resolution_y = 1080;

size_t Options::UPS_density = 800;

double Options::UPS_TITLE = 0.07;
double Options::UPS_default_height_ratio = 0.04;

}
