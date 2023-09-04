#include "polyscope/polyscope.h"

#include "../UPS/UnifiedPresentationSystem.h"
#include "imgui.h"

using namespace UPS;
UPS::Slideshow show(true);

PrimitiveID explorer_id;

vec point_explore(scalar t) {
    return vec(cos(t),sin(t),0)*0.1;
}

vec phi(vec x) {
    return vec(x(0),x(1),std::sin(x(0))*std::sin(x(1)));
}

vec phi_offset(vec x) {
    return phi(x) + vec(1.5,0,0);
}

vec sphere(vec x) {
    auto t1 = x(0)*M_PI_2;
    auto t2 = x(1)*M_PI;
    return vec(
                -sin(t1),
                cos(t1)*sin(t2),
                cos(t1)*cos(t2)
                );
}
vec sphere_offset(vec x) {
    return sphere(x) + vec(1.5,0,0);
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
    show << "Intro";

    if (false)
    {
        auto title = Title("Calcul différentiel et approximation de Taylor")->at(TOP);
        show << newFrame << title;
        show << "Taylor";
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
        show << newFrame << title << Latex::Add("Idée générale des approches différentielles :\\\\ Approcher localement un objet complexe par un objet simple (linéaire)")->at(CENTER);
    }

    if (true)
    {
        show << "manifold";
        auto title = Title(tex::center("Variétés différentielles et paramétrisation"));
        show << newFrame << title;
        show << inNextFrame << title->at(TOP);
        mapping offset = [](vec x){
            x(0) += -1.5;
            return x;
        };

        auto grid_param = grid->apply(offset);
        grid_param->pc->setEdgeWidth(1);
        auto cam = CameraView::Add(vec(0,0.6,5),vec(0,0.6,0),vec(0,1,0));
        show << cam;
        show << grid_param;
        auto arrow = Latex::Add(tex::equation("\\longrightarrow"));
        auto varphi = Latex::Add(tex::equation("\\varphi"));
        auto mani = grid->apply(sphere_offset);
        mani->pc->setEdgeWidth(1);
        auto arrowp = arrow->at(0.5,0.65);
        show << inNextFrame << mani << arrowp << PlaceBelow(varphi);

        PrimitiveGroup manifold;
        manifold << grid_param << mani << arrowp << cam;

        auto circle = [](scalar t){return vec(0.3*cos(TAU*t) + 0.2,0.3*sin(TAU*t),0);};

        auto curveParam = Curve3D::Add(circle,100,true);
        curveParam->setRadius(0.008);
        auto circle_slow = [](scalar t){return vec(0.3*cos(t) + 0.2,0.3*sin(t),0);};
        auto point_param = UPS::Point::Add(circle_slow);

        show << inNextFrame <<
            curveParam->apply(offset,true)
             << curveParam->apply(sphere_offset,true)
             << point_param->apply(offset)
             << point_param->apply(sphere_offset);

        auto paramdef = Latex::Add(tex::center(
                           "Une \\textit{paramétrisation} est une fonction $\\varphi$\n" +
                           tex::equation("\\varphi : \\mathbb{R}^2 \\longrightarrow \\mathcal{M}")
                           ));
        show << inNextFrame << paramdef->at(0.5,0.8);

        {
            StateInSlide t = arrowp.second;
            t.angle = M_PI;

            show << inNextFrame >> title << Title("Paramétrisation inverse ?")->at(TOP) << arrow->at(t);
            show << newFrame <<
                Title("Vecteurs tangents")->at(TOP) << manifold << mani->at(0.7);
            auto P = Point::Add(vec(0,0,0));
            auto step = [] (scalar t) {
                return periodic01(0.3*t)*0.3+0.01;
            };
            auto dx = [step](scalar t){
                return vec(step(t),0,0);
            };
            auto step_dx = Point::Add(dx);
            auto Pdx = step_dx->apply(offset);
            auto dy = [step](scalar t){
                return vec(0,step(t),0);
            };
            auto step_dy = Point::Add(dy);
            auto Pdy = step_dy->apply(offset);
            auto pp = P->apply(offset);
            auto pm = P->apply(sphere_offset);
            show << pp << pm;
            show << inNextFrame;
            show << Pdx << step_dx->apply(sphere_offset) << pm->addVector([Pdx](scalar t){
                auto X = sphere(Pdx->getCurrentPos()+vec(1.5,0,0));
                auto O = sphere(vec(0,0,0));
                vec rslt = 0.5*(X-O).normalized();
                return rslt;
        });
            auto dpx = Latex::Add(tex::frac("\\varphi(p+"+tex::Vec("dx","0")+")-\\varphi(p)","dx"),true);
            show << PlaceLeft(dpx,0.2);
            show << inNextFrame >> dpx;
            auto dpy = Latex::Add(tex::frac("\\varphi(p+"+tex::Vec("0","dy")+")-\\varphi(p)","dy"),true);
            show << PlaceLeft(dpy,0.2);
            show << Pdy << step_dy->apply(sphere_offset) << pm->addVector([Pdy](scalar t){
                auto X = sphere(Pdy->getCurrentPos()+vec(1.5,0,0));
                auto O = sphere(vec(0,0,0));
                vec rslt = 0.5*(X-O).normalized();
                return rslt;
        });
            {
                using namespace tex;
                show << newFrame << Title("Espace tangent")->at(TOP) << manifold;
                std::string parphi = "\\partial \\varphi";
                auto delx = frac(parphi,"\\partial x");
                auto dely = frac(parphi,"\\partial y");
                auto plane = Latex::Add("T\\mathcal{M}_p = \\text{Vect}(" + delx+"(p),"+dely+"(p))",true);
                auto P = Point::Add(vec(0,0,0))->apply(offset);
                auto po = point_param->apply(offset);
                auto ps = point_param->apply(sphere_offset);
                show << PlaceLeft(plane,0.3) << po << ps;
                auto TM =  grid->applyDynamic([po](vec x,TimeTypeSec){
                    vec p = po->getCurrentPos() + vec(1.5,0,0);
                        auto h = 1e-3;
                    vec dx = (sphere(p+vec(h,0,0))-sphere(p)).normalized()*0.4;
                    vec dy = (sphere(p+vec(0,h,0))-sphere(p)).normalized()*0.4;
                    vec t = sphere_offset(p) + dx*x(0) + dy*x(1);
                    return t;
            });
                TM->pc->setEdgeWidth(1);
                show << TM;
                show << inNextFrame << po->addVector([](scalar){
                        vec rslt= vec(2,3,0)*0.1;
                        return rslt;
            }) << ps->addVector([po](scalar){
                    vec p = po->getCurrentPos() + vec(1.5,0,0);
                        auto h = 1e-3;
                    vec dx = (sphere(p+vec(h,0,0))-sphere(p)).normalized()*0.4;
                    vec dy = (sphere(p+vec(0,h,0))-sphere(p)).normalized()*0.4;
                    vec t = (dx*2 + dy*3)*0.2;
                    return t;

            });
                show << PlaceRight(Latex::Add("v \\longrightarrow \\begin{pmatrix} " + delx + "(p)," + dely +"(p)\\end{pmatrix} v = J_{\\varphi}(p)v",true),0.3,0.1);
            }
        }
        {
            std::string J = "J_{\\varphi}(p)";
            show << newFrame;// << Title("Tenseur métrique");
            auto dot = Latex::Add("u \\cdot v = u^t v",true)->at(0.5,0.4);
            show << dot;
            auto Jdot = Latex::Add(J + "u \\cdot " + J + " v = ("+ J + "u)^t" + J + "v",true);
            auto arrowp = PlaceLeft(arrow);
            show << inNextFrame << arrowp << PlaceAbove(varphi) << PlaceABelowB(Latex::Add("p",true),arrowp) << PlaceABelowB(Jdot,dot,0.05);
            auto g = Latex::Add("= u^t" + tex::transpose(J) + J + "v = u^t g(p) v",true);
            show << inNextFrame << PlaceBelow(g,0.05) << Title("Tenseur métrique")->at(TOP);
            show << newFrame << Title("Tenseur métrique")->at(TOP) << PlaceBelow(Latex::Add("Disque de poincaré",false,0.05));
        }


    }

    if (false)
    {
        auto title = Latex::Add(tex::center("Représentation discrète des formes et fonctions"));
        show << newFrame << title;
        show << inNextFrame << title->at(TOP);
    }

    if (false)
    {
        show << "gradient";
        auto f = [](const vec& X) {
            auto x = X(0);auto y = X(1);
            return x*x+y*y;
        };
        auto gradf = [](const vec& X) {
            return 2*X;
        };
        auto lift = [f](const vec& X) {
            return vec(X(0),X(1),f(X));
        };
        auto F = grid->eval(f);
        auto GF = grid->eval(gradf);

        using namespace tex;
        auto title = Title("Retour sur le gradient : $\\nabla$");
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
        show << inNextFrame  << CameraView::Add(vec(0,0.5,4),vec(0,0.5,0),vec(0,1,0),true);
        show.getCurrentSlide().remove(gl);

        auto fval = grid_edge->pc->addVertexScalarQuantity("f000",F);
        fval->setColorMap("jet");
        auto gfval = grid_edge->pc->addVertexVectorQuantity("V000",GF);
        gfval->setVectorRadius(0.01,false);

        show << PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(fval);
        show << inNextFrame << PlaceRight(Latex::Add("\\nabla f(x,y) = "+ tex::Vec("x","y"),true),0.5);
        show << PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::Add(gfval);
    }

}



int main(int argc,char** argv) {
    show.init(UPS_prefix + "../scripts/course_MG.txt");
    init();
    const auto& S = show;
    int a = 2;

    polyscope::state::userCallback = [](){
        show.play();
    };
    polyscope::show();
    return 0;
}
