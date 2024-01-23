#include "polyscope/polyscope.h"

#include "../../src/UnifiedPresentationSystem.h"
#include "imgui.h"

using namespace UPS;
UPS::Slideshow show;


void init () {

    show << Title("TEST")->at(TOP);
    show << PlaceBelow(Latex::Add("subtitle"));
}



int main(int argc,char** argv) {

    show.init("dev");
    init();

    polyscope::state::userCallback = [](){
        show.play();
    };
    polyscope::show();
    return 0;
}
