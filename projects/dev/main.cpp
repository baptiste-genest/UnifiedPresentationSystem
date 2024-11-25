#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"
#include "spdlog/spdlog.h"


using namespace slope;
slope::Slideshow show;


void init () {
    LatexLoader::Init(Options::ProjectPath + "test.json");

    Primitive::DefaultTransition = SlideInSlideOut();

    auto M = Mesh::Add(slope::Options::DataPath + "meshes/torus.obj",1,true);

    M->updater = [M] (TimeObject time,Primitive*) {
        scalar t = time.from_begin*2;
        M->localTransform = Transform::ScalePositionRotate(0.75 + 0.25*sin(t),vec(0,0,sin(t)*0.2),vec(0,1,0),t);
    };

    show << Title("yoo") << newFrame << M->at(-2,0,0) << inNextFrame << M->at(2,0,0);
    show << newFrame;
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
