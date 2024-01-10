#include "polyscope/polyscope.h"

#include "../../src/UnifiedPresentationSystem.h"
#include "imgui.h"
using namespace UPS;

Slideshow show;


vec phi(const vec& x){
    return vec(x(0) + 3.,std::exp(-7*x.squaredNorm()),x(2));
}

vec catenoid(const vec& x){
    auto u = x(0);
    auto v = x(1)*2*M_PI;
    auto a = 1.;
    return vec(
        a*std::cosh(u)*std::cos(v),
        a*std::cosh(u)*std::sin(v),
        a*u
        );
}

vec circle(scalar t){
    t*= 2*M_PI;
    return vec(
        std::cos(t),
        std::sin(t),
        0
        );
}

std::function<scalar(scalar)> buildRangeMapper(scalar a,scalar b,scalar c,scalar d) {
    return [a,b,c,d] (scalar x) {
        return c + (d-c)*(x-a)/(b-a);
    };
}
mapping offset(const vec& x){
    return [x] (const vec& X){return X+x;};
}

void init () {

    using namespace UPS;
    auto CMFONTID = FontManager::addFont(Options::DataPath + "fonts/ComputerModernSR.ttf",80);
    auto grid = Mesh::Add(Options::DataPath + "meshes/grid_quad_50.obj");
    auto quad = Mesh::Add(Options::DataPath + "meshes/quad.obj");
    auto cube = Mesh::Add(Options::DataPath + "meshes/cube.obj");
    auto bunny = Mesh::Add(Options::DataPath + "meshes/bunny.obj");
    auto bunny_coarse = Mesh::Add(Options::DataPath + "meshes/bunny_coarse.obj");


    Latex::DeclareMathOperator("argmin","arg\\,min");
    Latex::DeclareMathOperator("area","Area");
    Latex::NewCommand("mass","\\text{mass}");

    Style::default_font = CMFONTID;

    Parametrization P(circle);

    if (true)
    {
        auto title = Title(tex::center("Computing Minimal Surfaces\\\\ using Differential forms"));
        show << title->at(CENTER);
        show << PlaceBelow(Latex::Add("Albert Cheng"));
        show << PlaceBelow(Latex::Add("Stephanie Wang"));
    }

    if (true)
    {
        auto cat_surf = grid->apply(catenoid);

        show << newFrame << Title("Minimal surface")->at(TOP);
        show << PlaceBelow(Latex::Add("Plateau's problem"));

        show << CameraView::Add(UPS::Options::ProjectViewsPath + "catenoid.json");
        show << inNextFrame;
        auto border = Formula::Add("\\Gamma",0.08);
        show << border->at("border");
        show << Curve3D::Add(P*std::cosh(1.)+vec(0,0,1),100,true);
        show << Curve3D::Add(P*std::cosh(1.)+vec(0,0,-1),100,true);

        show << inNextFrame << cat_surf;
        show << PlaceRelative(Formula::Add("\\rightarrow \\Sigma",0.08),border,placeX::REL_RIGHT,placeY::SAME_Y);
        show << PlaceBelow(Formula::Add("\\Sigma \\in \\text{argmin Area}(\\Sigma) "));
        show << PlaceBelow(Formula::Add("\\partial \\Sigma = \\Gamma "));
    }
    if (true)
    {
        show << newFrame << Title("k-vectors")->at(TOP);
        auto algebra = Formula::Add("\\bigwedge V");
        show << PlaceNextTo(algebra,1,0.1);
        show << CameraView::Add(UPS::Options::ProjectViewsPath + "kvectors.json");
        auto origin = Point::Add((vec)vec::Zero(),0.01);
        vec u(1,0,0);
        vec v(0,1,0);
        show << inNextFrame << origin << origin->addVector([u](scalar) {return u;});
        show << origin->addVector([v](scalar){return v;});

        auto uv =  Formula::Add("u,v");
        auto uwv =  Formula::Add("u\\land v");
        show << uv->at(0.35,0.5);
        show << inNextFrame << Replace(uwv);

        show <<  quad->apply([u,v](const vec& x){
            return vec(u*(x(0)+1)*0.5 + v*(x(1)+1)*0.5 + vec(0,0,-0.01));
        });
        auto anti_comm = Formula::Add("u\\land v = -v \\land u");
        show << inNextFrame << PlaceRight(anti_comm,0.35);


        show << inNextFrame << Title("k-forms")->at(TOP);
        show << PlaceBelow(Formula::Add("\\omega(u\\land v) = -\\omega(v\\land u) \\rightarrow \\mathbb{R}"),anti_comm,0.05);
        show << Replace(Formula::Add("\\bigwedge V^*"),algebra);

        vec w(-0.2,0.3,0.5);

        auto T = [u,v,w] (const Vec& x) {
            return x(0)*u + x(1)*v + x(2)*w;
        };

        show << inNextFrame;
        show << Latex::Add("multilinear and alternating $\\rightarrow$ det")->at(0.75,0.6);
        show << uwv->at(0.35,0.75);
        show << inNextFrame << cube->apply(T)->at(0.5);
        show << origin->addVector([w](scalar) {return w;});
        show << PlaceBelow(Formula::Add("\\omega(u\\land v) = \\text{det}(u,v,w)"),uwv);

    }
    if (true)
    {
        auto title = Title("Differential k-forms");
        show << newFrame << title->at(TOP);
        auto codim = Image::Add(UPS::Options::ProjectPath + "codim.png");
        auto ext_d = Image::Add(UPS::Options::ProjectPath + "ext_der.png");
        show << inNextFrame << codim->at(CENTER);
        show << newFrameSameTitle;
        show << PlaceBelow(Latex::Add("exterior derivative"),title);
        show << inNextFrame << ext_d->at(0.8,0.5);
        show << Image::Add(UPS::Options::ProjectPath + "d_boundary.png")->at(0.3,0.5);
    }
    // TODO: SHOW EQUIVALENCE BETWEEN ONE FORMS AND VECTOR FIELDS
    if (true) {
        show << newFrame << Title("How to represent a shape in a computer")->at(CENTER) << inNextFrame << TOP;
        show << CameraView::Add(UPS::Options::ProjectViewsPath + "shape.json");
        scalar off = 3;
        auto bco =bunny_coarse;
        bco->pc->setEdgeWidth(1.);
        bco->pc->setSmoothShade(false);
        show << bco;
        show << PointCloud::Add(bunny_coarse->getVertices())->apply(offset(vec(-off,0,0)));
        auto SDF = grid->eval([](const vec& x) {return -(x.norm()-0.6);});
        auto implicit = grid->scale(1.5)->translate(vec(off,0.8,0));
        show << Latex::Add("explicit")->at(0.4,0.9);
        show << inNextFrame << implicit << Latex::Add("implicit")->at(0.75,0.9);
        show << inNextFrame <<  Curve3D::Add(P*0.6*1.5 + vec(off,0.8,0),100,true);
        implicit->pc->addVertexSignedDistanceQuantity("sdf",SDF)->setEnabled(true);
    }

   auto surface = grid->apply([](const vec& X){
            auto x = X(0)*0.5;
            auto y = X(1)*0.5;
            return vec(x,y,std::exp(x*x+y*y)-1);
        });

    if (true) {
        show << newFrame << Title("Currents")->at(TOP);
        show << PlaceBelow(Latex::Add(tex::center("Currents are to differential forms \\\\ what distributions are to $\\mathcal{C}^{\\infty}$ functions")),0.1);
        show << CameraView::Add(UPS::Options::ProjectViewsPath + "currents.json");
        show << inNextFrame << surface;
        show << PlaceRight(Formula::Add("\\langle \\delta_{\\Sigma},\\omega \\rangle = \\int_{\\Sigma} \\omega "));
        show << inNextFrame << PlaceBelow(Formula::Add("d\\delta_{\\Sigma}"),0.03);
        show << inNextFrame << PlaceBelow(Formula::Add("\\langle d\\delta_{\\Sigma},\\omega \\rangle"),0.03);
        auto domega = Formula::Add("=\\langle \\delta_{\\Sigma},d\\omega \\rangle");
        show << inNextFrame << Replace(domega);
        show << inNextFrame << domega->at("d_omega");
        show << PlaceNextTo(Formula::Add("= \\int_{\\Sigma} d\\omega "),1);
        show << inNextFrame << PlaceBelow(Formula::Add("\\stackrel{\\mathclap{\\tiny\\mbox{Stokes}}}{=} \\int_{\\partial\\Sigma} \\omega"));
        show << inNextFrame << Replace(Formula::Add("= \\int_{\\Gamma} \\omega "));
        show << inNextFrame << Replace(Formula::Add("= \\langle \\delta_{\\Gamma},\\omega \\rangle"));
        show << inNextFrame << Replace(Formula::Add("d\\delta_{\\Sigma}= \\delta_{\\Gamma}"));
    }
    if (true) {
        show << newFrame << Title("Currents")->at(TOP);
        show << PlaceBelow(Latex::Add("Link with area"));
        show << Latex::Add("Area$(\\Sigma)$ ")->at(0.4,0.3);
        show << inNextFrame << PlaceNextTo(Formula::Add("=\\int_{\\Sigma} 1 dS"),1);
        show << inNextFrame << PlaceBelow(Formula::Add("=\\int_{\\Sigma} ||n(x)||^2 dS"));
        show << inNextFrame << PlaceBelow(Formula::Add("= \\max_{v \\text{ s.t.} ||v||_{\\infty} \\leq 1} \\int_{\\Sigma} v(x)\\cdot n(x) dS"));
        show << inNextFrame << PlaceBelow(Formula::Add("= \\max_{v \\text{ s.t.} ||v||_{\\infty} \\leq 1} \\langle \\delta_{\\Sigma} , v\\rangle"));
        show << inNextFrame << PlaceBelow(Formula::Add("= ||\\delta_{\\Sigma}||_{\\text{dual}}"));
        show << inNextFrame << Replace(Formula::Add("= ||\\delta_{\\Sigma}||_{\\text{mass}}"));
    }
    if (true)
    {
        show << newFrame << Title("The article's approach")->at(TOP);
        show << inNextFrame << Formula::Add("\\argmin_{\\Sigma : \\partial \\Sigma = \\Gamma} \\area(\\Sigma)")->at(0.3,0.25);
        show << inNextFrame << PlaceNextTo(Formula::Add(" = \\argmin_{\\Sigma : d \\delta_{\\Sigma} = \\delta_\\Gamma} ||\\delta_\\Sigma||_{\\mass}"),1);
        show << CameraView::Add(UPS::Options::ProjectViewsPath + "currents.json");
        show << surface;
        show << inNextFrame << PlaceNextTo(Formula::Add("= \\argmin_{\\eta : d \\eta = \\delta_\\Gamma} ||\\eta||_{1}"),1);
        show << inNextFrame << CameraView::Add(UPS::Options::ProjectViewsPath + "field.json");
        show >> surface;
        int N = 6;
        auto coord = buildRangeMapper(0,N-1,-0.5,0.5);
        auto l = 0.5/N;
        for (int k = 0;k<N;k++)
            for (int j = 0;j<N;j++)
                for (int i = 0;i<N;i++) {
                    auto field = [i,j,k,l,coord] (const Mesh::Vertex& v,const TimeObject& t) {
                        auto x = coord(i);
                        auto y = coord(j);
                        auto z = coord(k);
                        auto X = vec(x,y,z);
                        auto h = v.pos;
                        if (t.relative_frame_number == 0)
                            return vec(X+h*l);
                        auto I = std::exp(-t.from_action);
                        auto sdf = exp(x*x+y*y)-z-1;
                        vec grad = vec(0,0,1)*I + (1-I)*vec(2*x*std::exp(x*x+y*y),2*y*std::exp(x*x+y*y),-1).normalized();
                        auto rp = CompleteBasis(grad);
                        h = h(0)*rp.first + h(1)*rp.second;
                        return vec(X+h*l*std::exp(-t.from_action*std::pow(sdf,2)*10));
                    };
                    auto kvec =  quad->applyDynamic(field);
                    kvec->pc->setSurfaceColor(glm::vec3(1.,0,0));
                    kvec->pc->setEdgeWidth(1.5);
                    show << kvec;
                }
        show << inNextFrame;
    }
    {
        show << newFrame << Title("Quick recap");

    }

}



int main(int argc,char** argv) {
    show.init("minimal_surf","",false);
    init();

    polyscope::state::userCallback = [](){
        //ImGui::ShowDemoWindow();
        show.play();
    };
    polyscope::show();
    return 0;
}
