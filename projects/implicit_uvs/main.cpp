#include "polyscope/polyscope.h"
#include "implicit_uvs_slides.h"

using namespace slope;

Slideshow show;


void init() {
    Latex::UsePackage("cmbright");

    LoadCommon();
    CreateIntroSlides(show);
    CreateContextSlides(show);
    CreateMovingSlides(show);
    CreateExpMapSlides(show);
    CreateSphereTracingSlides(show);
    CreateLogMapSlides(show);
    CreateCurveBasedSlides(show);
    CreateMultipleSeedsSlides(show);
}

int main(int argc,char** argv)
{
  show.init("implicit_uvs",argc,argv);
  
  init();
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}
