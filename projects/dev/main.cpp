#include "polyscope/polyscope.h"

#include "../../src/UnifiedPresentationSystem.h"
#include "imgui.h"

using namespace UPS;
UPS::Slideshow show;

scalar f(scalar x) {
    return std::sin(1./x);
}


void init () {

    show << Title("TEST")->at(TOP);
    show << PlaceBelow(Latex::Add("subtitle"));
    Figure F;
    F.PlotFunction(0.01,3,f,1000);
    show << Figure::Add(F);
}



int main(int argc,char** argv) {

    show.init("dev");
    init();

    polyscope::state::userCallback = [](){
        //ImGui::ShowDemoWindow();
        show.play();
    };
    polyscope::show();
    return 0;
}
