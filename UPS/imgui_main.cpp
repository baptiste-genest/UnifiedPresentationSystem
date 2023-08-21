#include "polyscope/polyscope.h"

#include "UnifiedPresentationSystem.h"
#include "imgui.h"

UPS::Slideshow show;

void init () {
    using namespace UPS;

    auto img = UPS::Image::LoadImage("../../data/lateX/formula.png");
    auto CMFONTID = UPS::FontManager::addFont("/home/eulerson314/dev/UnifiedPresentationSystem/data/fonts/ComputerModernSR.ttf",60);

    UPS::Style::default_font = CMFONTID;

    Slide S1;
    S1.add(img,Vec2(0.5,0.5));

    auto txt = Text::CreateText("Introduction to Discrete Differential Geometry");

    Slide S2;
    S2.add(txt,Vec2(0.5,0.1));
    vecs P(50);
    for (auto& p : P)
        p = vec::Random();
    S2.add(PointCloud::AddPointCloud(P),Vec2(0.5,0));

    Slide S3;
    S3.add(txt,Vec2(0.5,0.75));
    S3.add(img,Vec2(0.5,0.5));

    show.addSlides(S1,S2,S3);
}

void myCallBack() {
    show.play();
}


int main(int argc,char** argv) {
    polyscope::options::buildGui = false;
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneEnabled = false;
    polyscope::options::giveFocusOnShow = false;
    polyscope::init();
    init();

    polyscope::state::userCallback = myCallBack;
    polyscope::show();
    return 0;
}
