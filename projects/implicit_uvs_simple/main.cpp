#include "polyscope/polyscope.h"
#include "implicit_uvs_slides.h"

using namespace slope;

Slideshow show;


void init() {
    Latex::UsePackage("cmbright");

    show.templater = [] (SlideManager& show,ScreenPrimitivePtr ptr) {
        show << PlaceRelative(ptr,ABS_LEFT,REL_BOTTOM,0.02,0.05);
    };

    LoadCommon();
    CreateIntroSlides(show);
    CreateContextSlides(show);
    CreateMovingSlides(show);
    CreateExpMapSlides(show);
    CreateLogMapSlides(show);
    CreateCurveBasedSlides(show);
    CreateMultipleSeedsSlides(show);
    CreateMergingFieldsSlides(show);
    CreateCompactSupportSlides(show);
    CreateAtlasComputeSlides(show);
    CreateDetailsSlides(show);
    CreateConclusionSlides(show);
}

int main(int argc,char** argv)
{
  show.init("implicit_uvs_simple",argc,argv);
  
  init();
  
  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}
