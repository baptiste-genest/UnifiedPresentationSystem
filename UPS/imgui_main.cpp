#include "polyscope/polyscope.h"

#include "UnifiedPresentationSystem.h"
#include "imgui.h"

UPS::Slideshow show;


UPS::vec phi(const UPS::vec& x){
    return UPS::vec(x(0) + 3.,1./(1. + x(0)*x(0) + x(2)*x(2)),x(2));
}

void init () {
    using namespace UPS;

    auto CMFONTID = UPS::FontManager::addFont(UPS_prefix + "fonts/ComputerModernSR.ttf",60);
    UPS::Style::default_font = CMFONTID;

    auto txt = Text::Add("Introduction à la Géométrie Différentielle Classique et Discrète ");

    show << txt->at(0.5,0.5);

    auto base = show.getCurrentSlide();

    auto M = Mesh::Add(UPS_prefix + "meshes/quad_grid_50.obj");

    show << inNextFrame << txt->at(0.5,0.1) << M;
    auto arrow = Latex::Add(tex::equation("\\longrightarrow"));
    show << inNextFrame << M->apply(phi) << arrow->at(0.5,0.5);


    auto circle = [](scalar t){return vec(0.5*cos(TAU*t),0,0.5*sin(TAU*t));};

    auto curveParam = Curve3D::Add(circle,100,true);
    auto circle_slow = [](scalar t){return vec(0.5*cos(t),0,0.5*sin(t));};
    auto point_param = UPS::Point::Add(circle_slow);


    show << inNextFrame << curveParam << curveParam->apply(phi,true) << point_param << point_param->apply(phi);


    auto paramdef = Latex::Add(tex::center(
                                   "Une \\textit{paramétrisation} est une fonction $\\varphi$\n" +
                                   tex::equation("\\varphi : \\mathbb{R}^2 \\longrightarrow \\mathcal{M}")
                                   ));
    show << inNextFrame << paramdef->at(0.5,0.8);

    StateInSlide t = Vec2(0.5,0.5);
    t.angle = M_PI;

    show << newFrame << Text::Add("Paramétrisation inverse ?")->at(0.5,0.1) << arrow->at(t);
}

void myCallBack() {

    show.play();
    //ImGui::ShowDemoWindow();
}


int main(int argc,char** argv) {
    polyscope::options::buildGui = false;
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneEnabled = false;
    polyscope::options::giveFocusOnShow = false;
    polyscope::view::upDir = polyscope::view::UpDir::ZUp;
    polyscope::init();
    init();

    polyscope::state::userCallback = myCallBack;
    polyscope::show();
    return 0;
}
