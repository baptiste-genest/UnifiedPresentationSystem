#include <polyscope/polyscope.h>
#include "../../src/UnifiedPresentationSystem.h"


UPS::Slideshow show(false);

int main(int argc,char** argv)
{
  
  //Setting the paths
  UPS::Options::UPS_pathCONVERT="/opt/homebrew/bin/";
  UPS::Options::UPS_pathPDFLATEX="/Library/TeX/texbin/";
  UPS::Options::UPS_pathPDFTEX="/Library/TeX/texbin/";
  UPS::Options::UPS_pathGS="/opt/homebrew/bin/";
  UPS::Options::UPS_pathPDFCROP="/Library/TeX/texbin/";
  
  //Init
  show.init();
  
  //Slideshow title
  show << UPS::Latex::Add("Hello 3D World!",UPS::TITLE)->at(UPS::CENTER);
  
  //New slide
  auto title = UPS::Title("The Bunny")->at(UPS::TOP);
  show << UPS::newFrame << title;
  
  auto bunny = UPS::Mesh::Add(UPS::Options::UPS_prefix + "meshes/bunny.obj");
  show << bunny;
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}

