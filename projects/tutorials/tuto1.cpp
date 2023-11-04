#include "polyscope/polyscope.h"
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
  
  
  show.init();
  
  //Slideshow title
  show << UPS::Latex::Add("Hello World!",UPS::TITLE)->at(UPS::CENTER);
  
  //New slide
  auto title = UPS::Title("The beauty of $\\pi$ ")->at(UPS::TOP);
  show << UPS::newFrame << title;
  
  //Plain latex forumla
  auto formula = UPS::Formula::Add("\\pi = 4 \\int_0^1 \\frac{1}{1+x^2}dx");
  show << formula;
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}
