#include "polyscope/polyscope.h"

#include "../../src/UnifiedPresentationSystem.h"
#include "imgui.h"

UPS::Slideshow show;


UPS::vec phi(const UPS::vec& x){
    return UPS::vec(x(0) + 3.,std::exp(-7*x.squaredNorm()),x(2));
}

void init () {
    using namespace UPS;

    auto CMFONTID = UPS::FontManager::addFont(Options::UPS_prefix + "fonts/ComputerModernSR.ttf",80);
    UPS::Style::default_font = CMFONTID;

    auto txt = Text::Add("Introduction à la Géométrie Différentielle Classique et Discrète ");

    //show << txt->at(CENTER);

    Figure F1;
    F1.PlotFunction(0,2*M_PI,[](scalar x) {return std::sin(x*5);});
    show << Figure::Add(F1)->at(CENTER);

    auto M = Mesh::Add(Options::UPS_prefix + "meshes/grid_quad_50.obj");

    show << inNextFrame << txt->at(TOP) << M;
    auto arrow = Latex::Add(tex::equation("\\longrightarrow"));
    auto varphi = Latex::Add(tex::equation("\\varphi"));
    show << inNextFrame << M->apply(phi) << arrow->at(CENTER) << PlaceBelow(varphi);


    auto circle = [](scalar t){return vec(0.5*cos(TAU*t) + 0.2,0,0.5*sin(TAU*t));};

    auto curveParam = Curve3D::Add(circle,100,true);
    auto circle_slow = [](scalar t){return vec(0.5*cos(t) + 0.2,0,0.5*sin(t));};
    auto point_param = UPS::Point::Add(circle_slow);


    show << inNextFrame << curveParam << curveParam->apply(phi,true) << point_param << point_param->apply(phi);


    auto paramdef = Latex::Add(tex::center(
                                   "Une \\textit{paramétrisation} est une fonction $\\varphi$\n" +
                                   tex::equation("\\varphi : \\mathbb{R}^2 \\longrightarrow \\mathcal{M}")
                                   ));
    show << inNextFrame << paramdef->at(0.5,0.8);

    StateInSlide t = Vec2(CENTER);
    t.angle = M_PI;

    show << newFrame << Text::Add("Paramétrisation inverse ?")->at(TOP) << arrow->at(t);
}



int main(int argc,char** argv) {

    show.init();
    init();

    polyscope::state::userCallback = [](){
        show.play();
    };
    polyscope::show();
    return 0;
}
