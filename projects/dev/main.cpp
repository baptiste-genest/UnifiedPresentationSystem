#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"
#include "spdlog/spdlog.h"

using namespace slope;
slope::Slideshow show;


void init () {
//    LatexLoader::Init(Options::ProjectPath + "test.json");
}



int main(int argc,char** argv) {

    show.init("dev",argc,argv);
    init();
    polyscope::view::upDir = polyscope::UpDir::YUp;

    polyscope::state::userCallback = [](){
//        ImPlot::ShowDemoWindow();
        show.play();
    };
    polyscope::show();
    return 0;
}
