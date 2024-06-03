#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"
using namespace UPS;

//#include <gif_lib.h>

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

mapping offset(const vec& x){
    return [x] (const vec& X){return X+x;};
}

vecs randomSmoothVectorField(int N,const slope::Mesh::MeshPtr& quad) {
    vecs result(N*N*N);
    for (int k = 0;k<N;k++)
        for (int j = 0;j<N;j++)
            for (int i = 0;i<N;i++)
                result[k*N*N+j*N+i]  = vec::Random();
    double step = 0.1;
    //iterative smoothing
    for (int iter = 0;iter<50;iter++) {
        for (int k = 0;k<N;k++)
            for (int j = 0;j<N;j++)
                for (int i = 0;i<N;i++) {
                    vec sum = vec::Zero();
                    int count = 0;
                    for (int dk = -1;dk<=1;dk++)
                        for (int dj = -1;dj<=1;dj++)
                            for (int di = -1;di<=1;di++) {
                                if (i+di<0 || i+di>=N || j+dj<0 || j+dj>=N || k+dk<0 || k+dk>=N)
                                    continue;
                                sum += result[(k+dk)*N*N+(j+dj)*N+(i+di)];
                                count++;
                            }
                    result[k*N*N+j*N+i] += step*(sum/count - result[k*N*N+j*N+i]);
                }
    }

    auto coord = buildRangeMapper(0,N-1,-0.5,0.5);
    auto l = 0.5/N;
    for (int k = 0;k<N;k++)
        for (int j = 0;j<N;j++)
            for (int i = 0;i<N;i++) {
                auto field = [i,j,k,l,N,coord,&result,quad] (const Vertex& v) {
                    auto x = coord(i);
                    auto y = coord(j);
                    auto z = coord(k);
                    auto X = vec(x,y,z);
                    auto h = v.pos;
                    vec grad = result[k*N*N+j*N+i];
                    auto rp = CompleteBasis(grad);
                    h = h(0)*rp.first + h(1)*rp.second;
                    return vec(X+h*l);
                };
                auto kvec =  quad->apply(field);
                kvec->pc->setSurfaceColor(glm::vec3(1.,0,0));
                kvec->pc->setEdgeWidth(1.5);
                show << kvec;
            }
    return result;
}

void shape_evolution(int N,const slope::Mesh::MeshPtr& quad) {
    auto coord = buildRangeMapper(0,N-1,-0.5,0.5);
    auto l = 0.5/N;
    for (int k = 0;k<N;k++)
        for (int j = 0;j<N;j++)
            for (int i = 0;i<N;i++) {
                auto field = [i,j,k,l,coord] (const Vertex& v,const TimeObject& t) {
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
                    return vec(X+h*l*std::exp(-std::min(t.from_action,4.f)*std::pow(sdf,2)*10));
                };
                auto kvec =  quad->applyDynamic(field);
                kvec->pc->setSurfaceColor(glm::vec3(1.,0,0));
                kvec->pc->setEdgeWidth(1.5);
                show << kvec;
            }
}
std::vector<vec> nodes,tangents;

vec knot(double t){
    using namespace std;
    vec x;
    /*
    x << sin(t) + 2*sin(2*t),
            cos(t) - 2*cos(2*t),
            -sin(3*t);
*/
    x << cos(t), 0, sin(t);
    return x;
}
std::pair<vec,vec> getOrtho(const vec &v)
{
    static vec axis1(M_PI,2,3);
    static vec axis2(M_PI,1,M_PI);
    auto b1 =v.cross(axis1);
    if ((b1).squaredNorm() < 1e-6){
        b1 = v.cross(axis2);
        std::cout << "CHANGE" << std::endl;
    }
    b1 = b1.normalized();
    return {b1,v.cross(b1)};
}


void sample_curve(int N,const slope::Parametrization& curve) {
    double dt = 2*M_PI/N;
    double t = 0;
    for (int i = 0;i<N;i++){
        nodes.push_back(curve(t));
        tangents.push_back((curve(t+dt)-curve(t)).normalized());
        t += dt;
    }
}
float a = 0.1;
vec BS_integrand(const vec& p,const vec& p1,const vec& p2,bool end){
    vec etap = p2 - p1;
    vec eta = (end ? p2 : p1);
    vec diff = eta - p;
    double denom = (etap.squaredNorm()*a*a + diff.cross(etap).squaredNorm())*sqrt(a*a + diff.squaredNorm());
    return diff.dot(etap)*diff.cross(etap)/denom;
}

vec biot_savard_field(const vec& p) {
    vec u = vec::Zero();
    for (int i = 0;i<nodes.size();i++)
        u += (BS_integrand(p,nodes[i],nodes[(i+1)%nodes.size()],true) -BS_integrand(p,nodes[i],nodes[(i+1)%nodes.size()],false))/nodes.size();
    return u/(4*M_PI);
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
    Latex::NewCommand("curl","\\text{curl}");
    Latex::NewCommand("Stokes","\\stackrel{\\mathclap{\\tiny\\mbox{Stokes}}}{=}");
    Latex::NewCommand("mass","\\text{mass}");
    Latex::NewCommand("VA","\\Vec{A}");

    Style::default_font = CMFONTID;

    Parametrization P(circle);

    {
        auto title = Title(tex::center("Computing Minimal Surfaces\\\\ using Differential forms"));
        show << title->at(CENTER);
        show << PlaceBelow(Latex::Add("Albert Chern"));
        show << PlaceBelow(Latex::Add("Stephanie Wang"));
        show << PlaceBelow(Latex::Add("SIGGRAPH 2021"),0.05);
    }

    {
        auto cat_surf = grid->apply(catenoid);

        show << newFrame << Title("Minimal surface")->at(TOP);
        show << PlaceBelow(Latex::Add("Plateau's problem"));

        show << CameraView::Add(slope::Options::ProjectViewsPath + "catenoid.json");
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
    {
        show << newFrame << Title("How to represent a shape in a computer?")->at(CENTER) << inNextFrame << TOP;
        show << CameraView::Add(slope::Options::ProjectViewsPath + "shape.json");
        scalar off = 3;
        auto bco =bunny_coarse;
        bco->pc->setEdgeWidth(1.);
        bco->pc->setSmoothShade(false);
        show << bco;
        show << PointCloud::Add(bunny_coarse->getVertices())->apply(offset(vec(-off,0,0)));
        auto SDF1 = grid->eval([](const vec& x) {return -(x.norm()-0.6);});
        show << Latex::Add("explicit")->at(0.4,0.8);


        auto implicit = grid->scale(1.5)->translate(vec(off,0.8,0));
        show << inNextFrame << implicit << Latex::Add("implicit")->at(0.75,0.8);
        auto sdf1 = AddPolyscopeQuantity(implicit->pc->addVertexSignedDistanceQuantity("sdf",SDF1));
        show << sdf1;
        auto boundary1 = Curve3D::Add(P*0.6*1.5 + vec(off,0.8,0),100,true);
        show << inNextFrame << boundary1;

        auto SDF2 = grid->eval([](const vec& x) {return -std::min((x-vec(0,-0.5,0)).norm()-0.3,(x-vec(0,0.5,0)).norm()-0.3);});
        auto sdf2 = AddPolyscopeQuantity(implicit->pc->addVertexSignedDistanceQuantity("sdf2",SDF2));
        show << inNextFrame >> sdf1 >> boundary1 << sdf2;
        show << inNextFrame << Curve3D::Add(P*0.3*1.5 + vec(off,0.8 + 0.75,0),100,true) << Curve3D::Add(P*0.3*1.5 + vec(off,0.8 - 0.75,0),100,true);
    }
    {
        show << newFrame << Title("How to approximate the area of a surface?")->at(TOP);
        show << inNextFrame << Image::Add("surface_approx.png");
    }
    {
        show << newFrame << Title("Quick introduction to differential forms");
        show << PlaceBelow(Latex::Add("A unified framework to express vector calculus"));


        show << newFrame << Title("k-vectors")->at(TOP);
        auto algebra = Formula::Add("\\bigwedge V");
        show << PlaceNextTo(algebra,1,0.1);
        show << CameraView::Add(slope::Options::ProjectViewsPath + "kvectors.json");
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
    {
        auto title = Title("Differential k-forms");
        show << newFrame << title->at(TOP);
        auto codim = Image::Add("codim.png");
        show << Latex::Add("smooth field of k-forms : $\\mathbb{R}^n \\mapsto \\bigwedge V^*$")->at("diff_forms");
        show << inNextFrame << codim->at(CENTER);
        show << Latex::Add("$x \\mapsto $Ker$(\\omega(x))$")->at("codim");
    }
    {
        auto title = Title("Example : differential 1-forms");
        show << newFrame << title->at(TOP);
        auto form = Formula::Add("\\omega(x) = f(x)dx + g(x)dy + h(x)dz");
        auto cam = CameraView::Add(Options::ProjectViewsPath + "1_form.json");
        show << cam << PlaceBelow(form);
        auto V = randomSmoothVectorField(7,quad);
        show << newFrameSameTitle << VectorField::AddOnGrid(V) << cam << PlaceBelow(form) << PlaceBelow(Formula::Add("\\simeq (f(x),g(x),h(x))^t"));
    }
    {
        auto ext_d = Image::Add("ext_der.png");
        show << newFrame << Title("The exterior derivative")->at(TOP);
        show << inNextFrame << ext_d->at(0.8,0.5);
        show << Image::Add("d_boundary.png")->at(0.3,0.5);
        show << inNextFrame << Formula::Add("\\int_{\\Sigma} d\\omega = \\int_{\\partial \\Sigma} \\omega")->at("stokes") << PlaceNextTo(Latex::Add("Stokes' Theorem : "),0);
    }
   auto surface = grid->apply([](const vec& X){
            auto x = X(0)*0.5;
            auto y = X(1)*0.5;
            return vec(x,y,std::exp(x*x+y*y)-1);
        });

    {
        show << newFrame << Title("Currents")->at(TOP);
        show << PlaceBelow(Latex::Add(tex::center("Currents are to differential forms \\\\ what distributions are to $\\mathcal{C}^{\\infty}_c$ functions")),0.05);
        show << inNextFrame << PlaceBelow(Image::Add("dirac_delta.png"));
        show << inNextFrame << Replace(Image::Add("currents.png"));
        show << PlaceNextTo(Formula::Add("\\langle \\delta_{\\Sigma} ,\\omega \\rangle = \\int_{\\Sigma} \\omega"),1);
    }
    {
        show << newFrameSameTitle;
        show << PlaceBelow(Latex::Add("Why using them theoretically \\\\ for minimal surface problems?"));
        show << inNextFrame << Latex::Add("Direct link with Area")->at("curr_area");
        show << inNextFrame << PlaceBelow(Formula::Add("||\\delta_\\Sigma||_{\\text{dual}} = \\max_{\\omega \\text{ s.t.} ||\\omega||_{\\infty} \\leq 1} \\langle \\delta_{\\Sigma} , \\omega \\rangle"));
        show << inNextFrame << PlaceBelow(Formula::Add(" = \\text{Area}(\\Sigma)"));

        show << inNextFrame << Latex::Add("Topology $\\iff$ Derivative")->at("curr_der");
        show << inNextFrame << PlaceBelow(Formula::Add("\\langle d\\delta_{\\Sigma},\\omega \\rangle=\\langle \\delta_{\\Sigma},d\\omega \\rangle"));
        show << inNextFrame << PlaceBelow(Formula::Add("\\Stokes\\langle \\delta_{\\partial \\Sigma},\\omega \\rangle \\implies d\\delta_{\\Sigma} = \\delta_{\\partial \\Sigma}"));
    }
    {
        show << newFrame << Title("Why so many dual objects???")->at(CENTER);
        show << inNextFrame << PlaceBelow(Latex::Add(tex::center("It is often easier to handle function \\\\ of objects than the objects themselves.")));
    }
    {
        show << newFrame << Title("The article's approach")->at(TOP);
        show << inNextFrame << Formula::Add("\\argmin_{\\Sigma : \\partial \\Sigma = \\Gamma} \\area(\\Sigma)")->at("opti");
        show << inNextFrame << PlaceNextTo(Formula::Add(" = \\argmin_{\\Sigma : d \\delta_{\\Sigma} = \\delta_\\Gamma} ||\\delta_\\Sigma||_{\\text{dual}}"),1);
        //show << CameraView::Add(slope::Options::ProjectViewsPath + "currents.json");
        show << CameraView::Add(slope::Options::ProjectViewsPath + "field.json");
        show << surface;
        show << inNextFrame >> surface;
        int N = 6;
        shape_evolution(N,quad);
        show << inNextFrame << PlaceNextTo(Formula::Add("= \\argmin_{\\eta : d \\eta = \\delta_\\Gamma} ||\\eta||_{1} ="),1);
        auto vf_opti = Formula::Add("\\argmin_{X : \\curl(X) = \\delta_\\Gamma} ||X||_{1}");
        show << PlaceNextTo(vf_opti,1);
        show << Latex::Add(" Implicit representation : extract $\\Sigma$ by considering $X$ as its normal field")->at(0.5,0.9);
        show << newFrame << Title("An optimization problem")->at(TOP) << vf_opti->at(CENTER);
    }
    {
        show << newFrame << Title("FFT and Topology")->at(TOP);
        show << inNextFrame << PlaceBelow(Latex::Add("Imposing Curl$(X) = v \\implies$ solving many $\\Delta X = Y$"),0.15);
        show << inNextFrame << PlaceBelow(Latex::Add("To speed things up : FFT!"));
        show << inNextFrame << PlaceBelow(Latex::Add(" Solving $\\Delta X = Y$ on a $n \\times n \\times n$ grid :"),0.1);
        show << PlaceBelow(Latex::Add(" Without FFT : $\\mathcal{O}(n^6)$"));
        show << PlaceBelow(Latex::Add(" With FFT : $\\mathcal{O}(n^3\\log(n))$"));
        show << newFrameSameTitle << PlaceBelow(Latex::Add("FFT $\\implies$ periodic domain"),0.1) << inNextFrame << Image::Add("T3.png")->at(CENTER);
    }
    {
        show << newFrame << Title("The Helmholtz-Hodge Decomposition")->at(TOP);
        show << Image::Add("helmholtz_hodge.png")->at("HH");
        show << inNextFrame << Formula::Add("v = \\alpha \\oplus \\beta \\oplus \\gamma")->at("HH_decomp");
        show << PlaceBelow(Formula::Add("\\alpha \\in \\text{Ker}(d^1) \\iff \\text{div}(\\alpha) = 0"));
        show << PlaceBelow(Formula::Add("\\beta \\in \\text{Im}(d^0) \\iff \\text{curl}(\\beta) = 0 \\iff \\beta = d\\varphi"));
        show << PlaceBelow(Formula::Add("\\gamma \\in \\frac{\\text{Ker}(d^1)}{\\text{Im}(d^0)}"));
        show << PlaceBelow(Latex::Add(tex::center("Allows us to decouple\\\\ the constraints !")));
    }
    {
        show << newFrame << Title("Imposing the boundary constraint")->at(TOP);
        show << CameraView::Add(slope::Options::ProjectViewsPath + "biot_savard.json");
        show << PlaceBelow(Latex::Add("The \\textit{Biot-Savard Law}"));
        Parametrization K(knot);
        int N = 40;
        sample_curve(N,K);
        auto curve = Curve3D::Add(nodes,true,0);
        show << curve;
        auto T = curve->pc->addNodeVectorQuantity("tangent",tangents);
        T->setVectorLengthScale(2*M_PI/N,false);
        T->setVectorRadius(0.04,false);
        show << AddPolyscopeQuantity(T);
        //random points
        vecs samples,V;
        for (int i = 0;i<1000;i++){
            samples.push_back(vec::Random()*1.5);
            V.push_back(biot_savard_field(samples.back()));
        }
        show << Latex::Add("Imposing $\\curl(X) = \\delta_{\\partial \\Sigma}$")->at("BS");
        show << PlaceBelow(Formula::Add("\\implies \\text{curl}^{+}(v) = \\text{curl}(\\Vec{\\Delta}^{-1}v)"));

        show << VectorField::Add(samples,V,0.3);
    }
    {
        show << newFrame << Title("Imposing the non-periodicity constraint")->at(TOP);
        show << PlaceBelow(Latex::Add("Fixing harmonic coordinates"));
        show << inNextFrame << Image::Add("H1_basis.png")->at("H1_basis") << PlaceBelow(Latex::Add("A basis of the set of harmonic vector fields"));
        show << inNextFrame << Formula::Add("H_i = \\langle X , e_i \\rangle &= \\int X_i d\\Omega \\\\ &=\\langle \\delta_{\\Sigma} , e_i \\rangle")->at("coords");
        show << inNextFrame << PlaceBelow(Formula::Add("\\langle \\delta_{\\Sigma} , e_i \\rangle \\Stokes (\\int_{\\partial \\Sigma} \\gamma \\times d\\gamma)_i"));
    }
    {
        show << newFrame << Title("Going back to our optimization problem")->at(TOP);
        show << Formula::Add("\\argmin_{X : \\curl(X) = \\delta_\\Gamma,H(X) = H(\\Sigma)} ||X||_{1}")->at(0.5,0.3);
        auto unconstrained = Formula::Add("\\argmin_{\\varphi} ||\\eta_0 + \\nabla \\varphi ||_1 \\text{, where } \\curl(\\eta_0) = \\delta_\\Gamma,H(\\eta_0) = H(\\Sigma)");
        show << inNextFrame << PlaceBelow(unconstrained,0.1);
        show <<  Title("Factorizing the constraints")->at(TOP);
        show << inNextFrame << Title("Final optimization Problem")->at(TOP);
        show << PlaceBelow(Formula::Add("\\argmin_{\\varphi,Y : Y = \\eta_0 + \\nabla \\varphi} ||Y||_1"),unconstrained,0.1);
        show << inNextFrame << PlaceBelow(Latex::Add("Minimizing a convex function under linear constraints $\\implies$ ADMM"),0.1);
    }
    {
        show << newFrame << Title("Conclusion")->at(TOP);
        show << PlaceLeft(Latex::Add(tex::enumerate("The article aims at an efficient computation of the solution",
                                          "To do so, they propose the use of the FFT",
                                          "This has theoretical consequences that they fully cover",
                                                    "The final optimization problem can be solved very efficiently"),Options::Slope_default_height_ratio*0.9),0.4) << Image::Add("mini_surf.png",0.7)->at("final");
    }
}



int main(int argc,char** argv) {
    show.init("minimal_surf",argc,argv);
    init();

    polyscope::state::userCallback = [](){
        //ImGui::ShowDemoWindow();
        show.play();
    };
    polyscope::show();
    return 0;
}
