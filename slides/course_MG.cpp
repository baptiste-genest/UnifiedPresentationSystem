#include "polyscope/polyscope.h"

#include "../UPS/UnifiedPresentationSystem.h"
#include "imgui.h"

using namespace UPS;
UPS::Slideshow show(true);

PrimitiveID explorer_id;

vec point_explore(scalar t) {
    return vec(cos(t),0,sin(t))*0.2;
}

vec phi(vec x) {
    return vec(x(0),std::sin(x(0))*std::sin(x(2)),x(2));
}

vec2 gradient(vec X) {
    auto x = X(0);
    auto y = X(2);
    return vec2(cos(x)*sin(y),cos(y)*sin(x));
}

mat2 hessian(vec X) {
    auto x = X(0);
    auto y = X(2);
    mat2 H;
    H << -sin(x)*sin(y), cos(x)*cos(y),cos(x)*cos(y),-sin(x)*sin(y);
    return H;
}
scalar zoffset = 0.02;

vec taylor2(vec H,TimeTypeSec) {
    auto t = Primitive::get(explorer_id)->getInnerTime();
    auto x = point_explore(t);
    auto g = gradient(x);
    vec2 h = vec2(H(0),H(2));
    auto Hf = hessian(x);
    auto p = phi(x);
    return p + H + vec(0,g.dot(h) + 0.5 * h.dot(Hf*h)+zoffset,0);
}

vec taylor1(vec H,TimeTypeSec) {
    auto t = Primitive::get(explorer_id)->getInnerTime();
    auto x = point_explore(t);
    auto g = gradient(x);
    vec2 h = vec2(H(0),H(2));
    auto p = phi(x);
    return p + H + vec(0,g.dot(h) +zoffset,0);
}


void init () {

    auto CMFONTID = UPS::FontManager::addFont(UPS_prefix + "fonts/ComputerModernSR.ttf",50);
    UPS::Style::default_font = CMFONTID;

    auto grid = Mesh::Add(UPS_prefix + "meshes/quad_grid_50.obj");
    auto disk = Mesh::Add(UPS_prefix + "meshes/disk_remesh.obj",vec(0.5,0.5,0.5));

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
        show << newFrame << title << Latex::Add("Idée générale des approches différentielles :\\\\ Approcher une objet complexe par un ensemble d'objets simples (linéaires)")->at(CENTER);
    }

    {
        auto f = [](const vec& X) {
            auto x = X(0);auto y = X(2);
            return x*x+y*y;
        };
        auto gradf = [](const vec& X) {
            auto x = X(0);auto y = X(2);
            return 2*X;
        };
        auto lift = [f](const vec& X) {
            return vec(X(0),f(X),X(2));
        };
        auto F = grid->eval(f);
        auto GF = grid->eval(gradf);

        using namespace tex;
        auto title = Latex::Add("Focus sur le gradient : $\\nabla$",false,0.07);
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
        show << inNextFrame  << CameraView::Add(vec(0,4,0.5),vec(0,0,0.5));
        show.getCurrentSlide().remove(gl);

        auto fval = grid->pc->addVertexScalarQuantity("f000",F);
        fval->setColorMap("jet");
        auto gfval = grid->pc->addVertexVectorQuantity("V000",GF);

        show << PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(fval);
        show << inNextFrame << PlaceRight(Latex::Add("\\nabla f(x,y) = "+ tex::Vec("x","y"),true),0.5);
        show << PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::Add(gfval);
    }

    {
    }
}



int main(int argc,char** argv) {
    polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
    show.init();
    init();

    polyscope::state::userCallback = [](){show.play();};
    polyscope::show();
    return 0;
}
