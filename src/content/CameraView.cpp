#include "CameraView.h"

UPS::CameraViewPtr UPS::CameraView::Add(const vec& f,const vec& t, const vec &up,bool flyTo)
{
  return std::make_shared<CameraView>(toVec3(f),toVec3(t),toVec3(up),flyTo);
}

UPS::CameraViewPtr UPS::CameraView::Add(std::string file, bool flyTo)
{
  file = formatCameraFilename(file);
  std::ifstream camfile(file);
  if (!camfile.is_open()){
    std::cerr << "invalid path " << file << std::endl;
    throw ;
  }
  std::string str((std::istreambuf_iterator<char>(camfile)),
                  std::istreambuf_iterator<char>());
  
  //Disable windowsHeight and windowsWidth
  std::string wh = "\"windowHeight\":";
  std::string ww = "\"windowWidth\":";
  std::string finalstr = str;
  
  std::size_t found = str.find(wh);
  if (found < str.size())
  {
    std::size_t sizeh = 0;
    while (((sizeh+found+wh.size())<str.size()) && (str[found + sizeh + wh.size()] != ',') && (str[found + sizeh+wh.size()] != '}')) sizeh++;
    finalstr.replace(found  + wh.size(), sizeh ,std::to_string(UPS::Options::UPS_screen_resolution_y));
  }
  
  std::size_t foundw = finalstr.find(ww);
  if (foundw < finalstr.size())
  {
    std::size_t sizew = 0;
    while (((sizew + foundw + ww.size()) < finalstr.size()) && (finalstr[foundw + sizew + ww.size()] != ',') && (finalstr[foundw + sizew + ww.size()] != '}')) sizew++;
    finalstr.replace(foundw + ww.size(), sizew ,std::to_string(UPS::Options::UPS_screen_resolution_x));
  }
  return std::make_shared<CameraView>(finalstr,flyTo);
}

std::string UPS::formatCameraFilename(std::string file)
{
  if (file[0] != '/'){
    file = Options::ProjectViewsPath + file + ".json";
  }
  return file;
}
