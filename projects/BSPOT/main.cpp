#include "polyscope/polyscope.h"
#include "../../src/slope.h"

#include "slides/bspot.h"
#include "slides/intro.h"

using namespace slope;

Slideshow show;

void init() {
    Latex::UsePackage("libertine");
    LatexLoader::Init(Options::ProjectPath + "latex.json");


    CreateIntroSlides(show);



}

int main(int argc,char** argv)
{
  show.init("BSPOT",argc,argv);
  
  init();
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}
