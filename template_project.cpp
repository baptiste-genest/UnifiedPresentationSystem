#include "polyscope/polyscope.h"
#include "../../src/UnifiedPresentationSystem.h"

using namespace UPS;

Slideshow show;

void init() {
  //Slideshow title
  show << Title("Hello World!")->at(UPS::CENTER);
  
  //New slide
  auto title = UPS::Title("The beauty of $\\pi$ ")->at(UPS::TOP);
  show << newFrame << title;
}

int main(int argc,char** argv)
{
  show.init("UPS_PROJECT_NAME");
  
  init();
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}
