#include "polyscope/polyscope.h"

#include "../UPS/UnifiedPresentationSystem.h"
#include "imgui.h"

using namespace UPS;
UPS::Slideshow show(false);

PrimitiveID explorer_id;

vec point_explore(scalar t) {
    return vec(cos(t),sin(t),0)*0.2;
}

vec phi(vec x) {
    return vec(x(0),x(1),std::sin(x(0))*std::sin(x(1)));
}

vec phi2(vec x) {
    return vec(x(0)+1.5,x(1),std::sin(x(0))*std::sin(x(1)));
}

vec2 gradient(vec X) {
    auto x = X(0);
    auto y = X(1);
    return vec2(cos(x)*sin(y),cos(y)*sin(x));
}

mat2 hessian(vec X) {
    auto x = X(0);
    auto y = X(1);
    mat2 H;
    H << -sin(x)*sin(y), cos(x)*cos(y),cos(x)*cos(y),-sin(x)*sin(y);
    return H;
}
scalar zoffset = 0.02;

vec taylor2(vec H,TimeTypeSec) {
    auto t = Primitive::get(explorer_id)->getInnerTime();
    auto x = point_explore(t);
    auto g = gradient(x);
    vec2 h = vec2(H(0),H(1));
    auto Hf = hessian(x);
    auto p = phi(x);
    return p + H + vec(0,0,g.dot(h) + 0.5 * h.dot(Hf*h)+zoffset);
}

vec taylor1(vec H,TimeTypeSec) {
    auto t = Primitive::get(explorer_id)->getInnerTime();
    auto x = point_explore(t);
    auto g = gradient(x);
    vec2 h = vec2(H(0),H(1));
    auto p = phi(x);
    return p + H + vec(0,0,g.dot(h) +zoffset);
}


void init () {

    auto CMFONTID = UPS::FontManager::addFont(UPS_prefix + "fonts/ComputerModernSR.ttf",50);
    UPS::Style::default_font = CMFONTID;

    auto grid = Mesh::Add(UPS_prefix + "meshes/grid_quad_50.obj");
    auto disk = Mesh::Add(UPS_prefix + "meshes/disk_coarse.obj",vec(0.5,0.5,0.5));
    polyscope::view::resetCameraToHomeView();

    show << Latex::Add("Les opérateurs différentiels sont vos amis.",false,TITLE)->at(CENTER);

    {
        auto title = Latex::Add("Philosophie générale : Approximation de Taylor",false,0.07)->at(TOP);
        show << newFrame << title;
        show << PlaceBelow(Latex::Add("f(x+h) = f(x) + f'(x) h + f''(x) \\frac{h^2}{2}  + o(h^2)",true),0.05);
        show << PlaceBelow(Latex::Add("f(x+h) = f(x) + h^t \\nabla f(x) + \\frac{1}{2} h^t H_f(x)h + o(||h||^2)",true),0.01);
        auto plot = grid->apply(phi);
        show << inNextFrame << plot->at(0.5);
        auto explorer = Point::Add(point_explore,0.1)->apply(phi);
        show << explorer;
        explorer_id = explorer->pid;

        auto S1 = show.getCurrentSlide();
        show << S1 << disk->applyDynamic(taylor1);
        show << S1 << disk->applyDynamic(taylor2);
        show << newFrame << title << Latex::Add("Idée générale des approches différentielles :\\\\ Approcher un objet complexe localement par un objet simple (linéaire)")->at(CENTER);
    }

    if (true)
    {
        auto title = Latex::Add(tex::center("Variétés différentielles et paramétrisation"));
        show << newFrame << title;
        show << inNextFrame << title->at(TOP);
        mapping offset = [](vec x){
            x(0) += -1.5;
            return x;
        };

        auto grid_param = grid->apply(offset);
        grid_param->pc->setEdgeWidth(1);
        show << CameraView::Add(vec(0,0,5),vec(0,0,0),vec(0,1,0));
        show << grid_param;
        auto arrow = Latex::Add(tex::equation("\\longrightarrow"));
        auto varphi = Latex::Add(tex::equation("\\varphi"));
        auto mani = grid->apply(phi2);
        mani->pc->setEdgeWidth(1);
        show << inNextFrame << mani << arrow->at(CENTER) << PlaceBelow(varphi);


        auto circle = [](scalar t){return vec(0.5*cos(TAU*t) + 0.2,0.5*sin(TAU*t),0);};

        auto curveParam = Curve3D::Add(circle,100,true);
        curveParam->setRadius(0.008);
        auto circle_slow = [](scalar t){return vec(0.5*cos(t) + 0.2,0.5*sin(t),0);};
        auto point_param = UPS::Point::Add(circle_slow);

        show << inNextFrame <<
                curveParam->apply(offset,true)
             << curveParam->apply(phi2,true)
             << point_param->apply(offset)
             << point_param->apply(phi2);

        auto paramdef = Latex::Add(tex::center(
                           "Une \\textit{paramétrisation} est une fonction $\\varphi$\n" +
                           tex::equation("\\varphi : \\mathbb{R}^2 \\longrightarrow \\mathcal{M}")
                           ));
        show << inNextFrame << paramdef->at(0.5,0.8);

        StateInSlide t = Vec2(CENTER);
        t.angle = M_PI;

        show << newFrame << Text::Add("Paramétrisation inverse ?")->at(TOP) << arrow->at(t);

    }

    {
        auto title = Latex::Add(tex::center("Représentation discrète des formes et fonctions"));
        show << newFrame << title;
        show << inNextFrame << title->at(TOP);
    }

    if (true)
    {
        auto f = [](const vec& X) {
            auto x = X(0);auto y = X(1);
            return x*x+y*y;
        };
        auto gradf = [](const vec& X) {
            auto x = X(0);auto y = X(1);
            return 2*X;
        };
        auto lift = [f](const vec& X) {
            return vec(X(0),X(1),f(X));
        };
        auto F = grid->eval(f);
        auto GF = grid->eval(gradf);

        using namespace tex;
        auto title = Latex::Add("Retour sur le gradient : $\\nabla$",false,0.07);
        show << newFrame << title->at(CENTER);
        show << inNextFrame << title->at(TOP);
        auto grad = Latex::Add("\\nabla f(x,y) = " + tex::Vec(frac("\\partial f","\\partial x")+"(x,y)",frac("\\partial f","\\partial y")+"(x,y)"),true);
        show << PlaceBelow(grad);
        auto grid_edge = DuplicatePrimitive(grid);
        grid_edge->pc->setEdgeWidth(1);
        show << inNextFrame << PlaceBelow(Latex::Add("Par exemple")) << grid_edge;
        auto S = show.getCurrentSlide();
        auto gl = grid_edge->apply(lift);
        show << S << gl;

        show << PlaceLeft(Latex::Add("f(x,y) = "+ frac("x^2+y^2","2"),true),0.5);
        show << inNextFrame  << CameraView::Add(vec(0,0.5,4),vec(0,0.5,0));
        show.getCurrentSlide().remove(gl);

        auto fval = grid_edge->pc->addVertexScalarQuantity("f000",F);
        fval->setColorMap("jet");
        auto gfval = grid_edge->pc->addVertexVectorQuantity("V000",GF);

        show << PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(fval);
        show << inNextFrame << PlaceRight(Latex::Add("\\nabla f(x,y) = "+ tex::Vec("x","y"),true),0.5);
        show << PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::Add(gfval);
    }

}



int main(int argc,char** argv) {
    show.init();
    init();

    polyscope::state::userCallback = [](){show.play();};
    polyscope::show();
    return 0;
}
