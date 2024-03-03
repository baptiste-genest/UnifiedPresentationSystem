#include "polyscope/polyscope.h"
#include "../../src/UnifiedPresentationSystem.h"


UPS::Slideshow show;

int main(int argc,char** argv)
{
  show.init("tutorials");
  
  //Slideshow title
  show << UPS::Latex::Add("Hello World!",UPS::Options::UPS_TITLE)->at(UPS::CENTER);
  
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
