#include "polyscope/polyscope.h"

#include "../UPS/UnifiedPresentationSystem.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"
#include "imgui.h"
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>

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


using complex = std::complex<double>;
complex toComp(const vec& x){
    return complex(x(0),x(1));
}
vec toVec(const complex& x){
    return vec(x.real(),x.imag(),0);
}

vec hyperbolic_geodesic(const vec& pv,const vec& qv,scalar t) {
    auto p = toComp(pv);
    auto q = toComp(qv);
    auto n = (q-p)/(1.-std::conj(p)*q);
    n /= std::abs(n);
    auto wn = n*std::tanh(t*0.5);
    return toVec((wn + p)/(1.+ std::conj(p)*wn));
}

mapping offset(const vec& x){
    return [x] (const vec& X){return X+x;};
}

#include "geometrycentral/numerical/linear_solvers.h"
Vec illustratePoissonProblem(std::string filename) {
    using namespace geometrycentral::surface;
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> position_geometry;
    std::tie(mesh, position_geometry) = readManifoldSurfaceMesh(UPS_prefix + "meshes/bunny_coarse.obj");
    ExtrinsicGeometryInterface& geometry = *position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexLumpedMassMatrix();
    SMat L = geometry.cotanLaplacian;
    Eigen::SimplicialLDLT<SMat> solver;
    solver.compute(L);
    std::cout << solver.info() << std::endl;
    SMat M = geometry.vertexLumpedMassMatrix;
    auto N = mesh->nVertices();
    Vec B = Vec::Ones(N);
    for (int i = 0;i<10;i++)
        //B.row(rand()%N) = ((vec::Random() + vec::Ones())*0.5).transpose();
        B(rand()%N) = polyscope::randomUnit();
    Vec rhs = M*B;
    return geometrycentral::solvePositiveDefinite(L,rhs);
    Mat X = Mat::Ones(N,3);
    for (int i = 0;i<3;i++){
        Vec rhs = M*B.col(i);
        Vec rslt = solver.solve(rhs);
        X.col(i) = rslt;
        std::cout << solver.info() << std::endl;
    }
    return X;
}

void init () {

    auto CMFONTID = UPS::FontManager::addFont(UPS_prefix + "fonts/ComputerModernSR.ttf",50);
    UPS::Style::default_font = CMFONTID;

    auto grid = Mesh::Add(UPS_prefix + "meshes/grid_quad_50.obj");
    auto disk = Mesh::Add(UPS_prefix + "meshes/disk_coarse.obj",0.5);
    auto bunny = Mesh::Add(UPS_prefix + "meshes/bunny.obj");
    auto bunny_coarse = Mesh::Add(UPS_prefix + "meshes/bunny_coarse.obj");
    polyscope::view::resetCameraToHomeView();

    auto top_cam = CameraView::Add(vec(0,0.6,5),vec(0,0.6,0),vec(0,1,0));

    show << Latex::Add("Les opérateurs différentiels sont vos amis.",TITLE)->at(CENTER);
    show << "Intro";

    if (true)
    {
        auto title = Title("Calcul différentiel et approximation de Taylor")->at(TOP);
        show << newFrame << title;
        show << "Taylor";
        show << PlaceBelow(Formula::Add("f(x+h) = f(x) + f'(x) h + f''(x) \\frac{h^2}{2}  + o(h^2)"),0.05);
        show << inNextFrame << PlaceBelow(Formula::Add("f(x+h) = f(x) + h^t \\nabla f(x) + \\frac{1}{2} h^t H_f(x)h + o(||h||^2)"),0.01);
        auto plot = grid->apply(phi);
        show << inNextFrame << plot;//->at(0.5);
        auto explorer = Point::Add(point_explore,0.1)->apply(phi);
        show << explorer;
        explorer_id = explorer->pid;

        auto S1 = show.getCurrentSlide();
        show << S1 << disk->applyDynamic(taylor1);
        show << S1 << disk->applyDynamic(taylor2);
        show << newFrame << title << Latex::Add("Idée générale des approches différentielles :\\\\ Approcher localement un objet complexe par un objet simple (polynomial)")->at(CENTER);
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
        show << top_cam;
        show << grid_param;
        auto arrow = Formula::Add("\\longrightarrow");
        auto varphi = Formula::Add("\\varphi");
        auto mani = grid->apply(sphere_offset);
        mani->pc->setEdgeWidth(1);
        auto arrowp = arrow->at(0.5,0.65);
        show << inNextFrame << mani << arrowp << PlaceBelow(varphi);

        PrimitiveGroup manifold;
        manifold << grid_param << mani << arrowp << top_cam;

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
            auto dpx = Formula::Add(tex::frac("\\varphi(p+"+tex::Vec("dx","0")+")-\\varphi(p)","dx"));
            show << PlaceLeft(dpx,0.2);
            show << inNextFrame >> dpx;
            auto dpy = Formula::Add(tex::frac("\\varphi(p+"+tex::Vec("0","dy")+")-\\varphi(p)","dy"));
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
                auto plane = Formula::Add("T\\mathcal{M}_p = \\text{Vect}(" + delx+"(p),"+dely+"(p))");
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
                show << PlaceRight(Formula::Add("v \\longrightarrow \\begin{pmatrix} " + delx + "(p)," + dely +"(p)\\end{pmatrix} v = J_{\\varphi}(p)v"),0.3,0.1);
            }
        }

        {
            std::string J = "J_{\\varphi}(p)";
            show << newFrame;// << Title("Tenseur métrique");
            auto dot = Formula::Add("u \\cdot v = u^t v")->at(0.5,0.4);
            show << dot;
            auto Jdot = Formula::Add(J + "u \\cdot " + J + " v = ("+ J + "u)^t" + J + "v");
            auto arrowp = PlaceLeft(arrow);
            show << inNextFrame << arrowp << PlaceAbove(varphi) << PlaceABelowB(Formula::Add("p"),arrowp) << PlaceABelowB(Jdot,dot,0.05);
            auto g = Formula::Add("= u^t" + tex::transpose(J) + J + "v");
            auto title = Title("Tenseur métrique")->at(TOP);
                         show << inNextFrame << PlaceBelow(g,0.05);
            show << inNextFrame << PlaceBelow(Formula::Add("g(p) = " + tex::transpose(J) + J,0.06),0.05) << title;
                         auto cam = CameraView::Add(vec(0,0.3,2),vec(0,0.3,0),vec(0,1,0));
            show << newFrame << title << PlaceBelow(Latex::Add("Disque de poincaré",0.05)) << cam << disk;
                             auto gval = disk->eval([](const vec& x) {
                return 1./std::pow(1-x.squaredNorm(),2);
            });
            auto gplot = disk->pc->addVertexScalarQuantity("norm",gval);
                             gplot->setColorMap("coolwarm");
            show << inNextFrame << Formula::Add("g(z) = " + tex::frac("1",tex::pow("(1-||z||^2)","2")) + tex::Mat<2,2>("1","0","0","1"))->at(0.5,0.3);
                             show << PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(gplot);
            show << inNextFrame;
            for (int i = 0;i<6;i++){
                vec p = point_explore(polyscope::randomUnit()*M_PI*2)*5;
                vec q = point_explore(polyscope::randomUnit()*M_PI*2)*5;
                auto gamma = [q,p](scalar t){
                    t = periodic01(t*0.15)*15;
                    vec rslt = hyperbolic_geodesic(q,p,t-5.)*0.5;
                    return rslt;
                };
                show << Point::Add(gamma,0.02);
            }
            show << newFrame << Title("Récap 1")->at(TOP);
            show << inNextFrame << PlaceLeft(Latex::Add("Approche différentielle : approximation locale par des polynômes"),0.4);
            show << inNextFrame << PlaceRelative(Latex::Add("Paramétrisation : fonction d'exploration de la variété"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Espace tangent : plan tangent à un point de la variété"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Tenseur métrique : changement dans le calul des angles et longueur sur la surface"),ABS_LEFT,REL_BOTTOM,0.1);
        }
    }

        {
            show << newFrame << Title("Représentation discrète des formes et fonctions")->at(CENTER) << inNextFrame << TOP;
            scalar off = 3;
                auto bunny_off = bunny->apply(offset(vec(off,0,0)));
            show << bunny_off;
            auto bco =bunny_coarse;
            bco->pc->setEdgeWidth(1.);
            bco->pc->setSmoothShade(false);
            show << bco;
            auto bunny_pc = PointCloud::Add(bunny_coarse->getVertices());
            auto bunny_pco = bunny_pc->apply(offset(vec(-off,0,0)));
            show << bunny_pco;
            auto cam = CameraView::Add(vec(-0.5,2,8),vec(-0.5,2,0),vec::UnitY());
            show << cam << inNextFrame >> bunny_pco >> bunny_off;
            show << newFrame << bco << cam << Title("Topologie d'un maillage")->at(TOP);
            show << inNextFrame << PlaceBelow(Formula::Add("V - E + F = 2(1-g)"),0.05);
            show << inNextFrame << PlaceBelow(Latex::Add(tex::center("On définit donc des fonctions sur des maillages par"))) <<
                                                                     PlaceBelow(Formula::Add("f : V \\longrightarrow \\mathbb{R}^n"));
            auto F = Vec::Random(bunny_pc->getPoints().size());
            auto bpc = DuplicatePrimitive(bunny_pc);
            bpc->pc->setPointRadius(0.03,false);
            auto Fq = PolyscopeQuantity<polyscope::PointCloudScalarQuantity>::Add(bpc->pc->addScalarQuantity("V0000",F));
            show << inNextFrame << bpc << Fq;
            show << inNextFrame << PlaceRight(Formula::Add(tex::Vec("1.21","0.32", "\\vdots", "5.2","3.24"),0.05),0.6,0.1);
        }

        {
            show << newFrame << Title("Discrétisation des opérateurs différentiels")->at(TOP);
            show << inNextFrame << PlaceLeft(Latex::Add(tex::center("Opérateurs différentiels classiques")));
            show << PlaceBelow(Formula::Add("\\partial (f + g) = \\partial f + \\partial g")) << inNextFrame << PlaceRelative(Formula::Add(tex::AaboveB("?","\\longrightarrow"),0.06),UPS::CENTER_X,SAME_Y);
            show << inNextFrame << PlaceRight(Latex::Add(tex::center("Opérateurs différentiels discrets")));
            show << PlaceBelow(Formula::Add("AX"));
        }

        {
            using namespace tex;
            show << newFrame << Title("Exemple : Le laplacien $\\Delta$")->at(TOP);
            auto df = "\\partial^2 f";
            auto lapdef = Formula::Add("\\Delta f = " + frac(df,del<2>(0)) + "+" + frac(df,del<2>(1)),0.06);
            show << inNextFrame << lapdef->at(CENTER);
            auto topolap = Latex::Add(equation("(Lf)_i = \\sum_{n \\in N_i}" + frac("f_n","|N_i|") + " - f_i"),0.06);
            show << inNextFrame >> lapdef << topolap->at(CENTER);
            show << inNextFrame << Vec2(0.5,0.3);
            show << PlaceLeft(Image::Add(UPS_prefix + "images/simple_graph.png"),0.6,0.2);
            show << PlaceRelative(Image::Add(UPS_prefix + "images/simple_lap_mat.png"),UPS::REL_RIGHT,SAME_Y);
            {
                show << newFrame << Title("Une discrétisation imparfaite ?")->at(TOP);
                                show << CameraView::Add(vec(2,0.5,4),vec(2,0.5,0),vec(0,1,0));
                    vecs X = {
                    vec(0,0,0),
                    vec(0,1,0),
                    vec(1,1,0)
                    };
                    vecs Y = {
                    vec(2,0,0),
                    vec(2,0.5,0),
                    vec(4,0.5,0)};
                    auto pcx = PointCloud::Add(X); pcx->pc->setPointRadius(.02);
                    auto pcy = PointCloud::Add(Y); pcy->pc->setPointRadius(.02);
                    show << inNextFrame << Curve3D::Add(X) << Curve3D::Add(Y) << pcx << pcy;
                    show << newFrame << Title("Discrétisation du laplacien par éléments finis")->at(TOP);
                    show << inNextFrame << PlaceBelow(Formula::Add("\\varphi_v(x_i) = " + cases("1","v=i","0",text("sinon")),0.05),0.05);
                    show << PlaceRelative(Latex::Add(center("Fonction linéaire \\\\ par morceau telle que")),UPS::REL_LEFT,UPS::SAME_Y);
                    auto diskverycoarse = Mesh::Add(UPS_prefix + "meshes/disk_very_coarse.obj",1.3);
                    diskverycoarse->pc->setEdgeWidth(1);
                    UPS::Vec fem_basis = UPS::Vec::Zero(diskverycoarse->getVertices().size());
                    auto i = rand()%diskverycoarse->getVertices().size();
                    fem_basis(i) = 1;
                    auto Q = PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(diskverycoarse->pc->addVertexScalarQuantity("FEM",fem_basis));
                    show << top_cam << diskverycoarse << Q;
                    vecs V = diskverycoarse->getVertices();
                    V[i] = vec(V[i](0),V[i](1),0.3);
                    diskverycoarse->updateMesh(V);
                    using namespace tex;

                    show << inNextFrame << PlaceRight(Formula::Add("A_{ij} = \\int_{\\Omega} \\nabla \\varphi_i(x)\\cdot\\nabla \\varphi_j(x) dx"));
                    show << newFrameSameTitle << PlaceBelow(Image::Add(UPS_prefix + "images/cotan_angles.png"),0.1);
                    show << PlaceBelow(Formula::Add("(Lf)_i =" + frac("1","A_i") + "\\sum_{(i,j)\\in E}"+frac("cot(\\alpha_{ij}) + cot(\\beta_{ij})","2")+"(f_i-f_j)"));
                    show << inNextFrame << PlaceRight(Image::Add(UPS_prefix + "images/cot_plot.png"));
            }
            show << newFrame << Title("Problème de Poisson")->at(TOP);
            //                    UPS::Vec C = illustratePoissonProblem("");
            //std::cout << C << std::endl;
            //                    auto PP = PolyscopeQuantity<polyscope::SurfaceColorQuantity>::Add(bunny_coarse->pc->addVertexColorQuantity("V0000",C));
            //auto PP = PolyscopeQuantity<polyscope::SurfaceScalarQuantity>::Add(disk->pc->addVertexScalarQuantity("V0000",C));
            //show << disk << PP;
            auto title = Title("Applications du laplacien");
            show << newFrame << title->at(TOP);
            show << PlaceBelow(Latex::Add("Energie de Dirichlet",0.05));
            show << inNextFrame << PlaceBelow(Formula::Add("E(f) = \\int_\\Omega ||\\nabla f||^2 dx",0.07),0.1);

            show << newFrame << title->at(TOP) << PlaceBelow(Latex::Add("Décomposition spectrale",0.05));
            show << inNextFrame << PlaceBelow(Formula::Add("\\Delta u = \\lambda u",0.07),0.1);
            //show << inNextFrame << PlaceBelow(Formula::Add("\\Delta u = 0",0.07),0.1);

        }

        if (false)
        {
            show << newFrame << Title("Courbures")->at(CENTER);
        }

    if (false)
    {
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
        auto grad = Formula::Add("\\nabla f(x,y) = " + tex::Vec(frac("\\partial f","\\partial x")+"(x,y)",frac("\\partial f","\\partial y")+"(x,y)"));
        show << PlaceBelow(grad);
        auto grid_edge = DuplicatePrimitive(grid);
        grid_edge->pc->setEdgeWidth(1);
        show << inNextFrame << PlaceBelow(Latex::Add("Par exemple")) << grid_edge;
        auto S = show.getCurrentSlide();
        auto gl = grid_edge->apply(lift);
        show << S << gl;

        show << PlaceLeft(Formula::Add("f(x,y) = "+ frac("x^2+y^2","2")),0.5);
        show << inNextFrame  << CameraView::Add(vec(0,0.5,4),vec(0,0.5,0),vec(0,1,0),true) >> gl;

        auto fval = grid_edge->pc->addVertexScalarQuantity("f000",F);
        fval->setColorMap("jet");
        auto gfval = grid_edge->pc->addVertexVectorQuantity("V000",GF);
        gfval->setVectorRadius(0.01,false);

        show << PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(fval);
        show << inNextFrame << PlaceRight(Formula::Add("\\nabla f(x,y) = "+ tex::Vec("x","y")),0.5);
        show << PolyscopeQuantity<polyscope::SurfaceVertexVectorQuantity>::Add(gfval);


    }

}



int main(int argc,char** argv) {
    //show.init(UPS_prefix + "../scripts/course_MG.txt");
    show.init();
    init();

    polyscope::state::userCallback = [](){
        show.play();
        //ImGui::ShowDemoWindow();
    };
    polyscope::show();
    return 0;
}
