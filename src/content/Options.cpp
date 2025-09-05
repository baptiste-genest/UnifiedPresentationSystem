#include "Options.h"



namespace slope
{

//std::string Options::SlopePath = "";

std::string Options::ProjectName;
std::string Options::ProjectPath;
std::string Options::CachePath;
std::string Options::ProjectDataPath;
std::string Options::ProjectViewsPath;

size_t Options::ScreenResolutionWidth = 1920;
size_t Options::ScreenResolutionHeight = 1080;

std::string Options::PathToPDFLATEX = "/usr/bin/pdflatex";
std::string Options::PathToCONVERT = "/usr/bin/convert";

size_t Options::PDFtoPNGDensity = 600;

double Options::TitleScale = 1.5;
double Options::DefaultLatexScale = 1.;

bool Options::ignore_cache = false;

Eigen::Vector3d Options::DefaultBackgroundColor = Eigen::Vector3d(1,1,1);

}
