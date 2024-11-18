#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"


using namespace slope;
slope::Slideshow show;


void init () {
    LatexLoader::Init(Options::ProjectPath + "test.json");

    show.templater = [] (SlideManager& show,ScreenPrimitivePtr ptr) {
        show << PlaceRelative(ptr,ABS_LEFT,REL_BOTTOM,0.02,0.05);
    };

 //   show << LatexLoader::Load("test");

    int n = 100;
    scalars X(n),Y(n);
    for (int i = 0;i<n;i++) {
        X[i] = scalar(i)/n;
        Y[i] = std::sin(X[i]*8);
    }
    show << Title("Plots now in slope!");
    show << newFrame << Plot::Add(PlotLines(X,Y))->at("testplot");
    show  << Plot::Add(PlotLines(X,Y));
    show  << Plot::Add(PlotLines(X,Y));
    show << newFrame << Title("Pretty sweet huh?");
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
