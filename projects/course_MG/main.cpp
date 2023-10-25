#include "polyscope/polyscope.h"

#include "../../src/UnifiedPresentationSystem.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"
#include "imgui.h"
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>

#include "geometrycentral/numerical/linear_solvers.h"

using namespace UPS;
UPS::Slideshow show(false);

PrimitiveID explorer_id;

vec point_explore(scalar t) {
    return vec(cos(t),sin(t),0)*0.2;
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

vec taylor(const Mesh::Vertex& v,TimeObject T) {
    auto& H = v.pos;
    auto t = Primitive::get(explorer_id)->getInnerTime();
    auto x = point_explore(t);
    vec2 h(H(0),H(1));
    auto g = gradient(x);
    auto p = phi(x);
    if (T.relative_frame_number == 0)
        return p + H + vec(0,0,g.dot(h) + zoffset);
    auto Hf = hessian(x);
    return p + H + vec(0,0,g.dot(h) + zoffset + 0.5 * h.dot(Hf*h)*smoothstep(T.from_action*0.5));
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

/*
std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> position_geometry;
*/
io::GeometryCentralMesh bunnyGC;

Mat illustratePoissonProblem() {
    Mat B;
    std::string cache = UPS_prefix + "cache/poisson.csv";
    if (io::MatrixCache(cache,B))
        return B;
    auto& mesh = bunnyGC;
    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexGalerkinMassMatrix();
    auto N = mesh.mesh->nVertices();
    SMat I(N,N);
    I.setIdentity();
    SMat L = geometry.cotanLaplacian + 1e-5*I;
    Eigen::SimplicialLDLT<SMat> solver(L);
    std::cout << solver.info() << std::endl;
    SMat M = geometry.vertexGalerkinMassMatrix;
    B = Mat::Zero(N,2);
    B(114) = 1;
    Vec rhs = M*B.col(0);
    B.col(1) = solver.solve(rhs);
    if (solver.info() != Eigen::Success)
        std::cout << "error eigen solve" << solver.info() << std::endl;
    io::SaveMatrix(cache,B);
    return B;
}

Mat posToMat(const vecs& X) {
    int N = X.size();
    Mat P(N,3);
    for (int i = 0;i<N;i++){
        P.row(i) = X[i].transpose();
    }
    return P;
}

vecs matToPos(const Mat& M) {
    int N = M.rows();
    vecs P(N);
    for (int i = 0;i<N;i++){
        P[i] = M.row(i).transpose();
    }
    return P;
}


Mat EigenLaplace(const io::GeometryCentralMesh& mesh,std::string label,int k) {
    Mat Eig;
    std::string cache = UPS_prefix + "cache/eig_"+label+ std::to_string(k) + ".csv";
    if (io::MatrixCache(cache,Eig))
        return Eig;

    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexGalerkinMassMatrix();
    geometry.requireVertexLumpedMassMatrix();
    auto N = mesh.mesh->nVertices();
    SMat I(N,N);
    I.setIdentity();
    SMat L = geometry.cotanLaplacian + 1e-5*I;
    auto E = geometrycentral::smallestKEigenvectorsPositiveDefinite(L,I,k,100);
    Eig = Mat(N,k);
    for (int i = 0;i<k;i++)
        Eig.col(i) = E[i].normalized();
    io::SaveMatrix(cache,Eig);
    return Eig;
}

vecs compute_eigen_coeffs(const Mat& eig,const vecs& pos) {
    vecs L(eig.cols());
    Mat P = posToMat(pos);
    for (int i = 0;i<eig.cols();i++){
        for (int j = 0;j<3;j++)
            L[i](j) = eig.col(i).dot(P.col(j));
    }
    return L;
}

vecs spectral_compression(const Mat& eig,const vecs& coefs,int nb) {
    Mat P = Mat::Zero(eig.rows(),3);
    for (int i = 0;i<nb;i++){
        for (int j = 0;j<3;j++){
            P.col(j) += coefs[i](j)*eig.col(i);
        }
    }
    return matToPos(P);
}


Vec LapHeight(){
    Mat LH;
    std::string cache = UPS_prefix + "cache/lap_height.csv";
    if (io::MatrixCache(cache,LH))
        return LH;


    io::GeometryCentralMesh mesh(UPS_prefix + "meshes/mountain.obj");
    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexGalerkinMassMatrix();
    auto N = mesh.mesh->nVertices();
    Vec H(N);
    for (const auto& v : mesh.mesh->vertices())
        H(v.getIndex()) = mesh.position_geometry->inputVertexPositions[v][1];
    SMat L = geometry.cotanLaplacian;// + 1e-5*I;
    LH = L*H;
    io::SaveMatrix(cache,LH);
    return LH;
}

vecs tutte_init(const io::GeometryCentralMesh& mesh,const vec& offset) {
    auto V = mesh.mesh->nVertices();
    vecs X(V,offset);
    for (const auto& v : mesh.mesh->vertices())
        if (v.isBoundary()){
            auto x =mesh.getPos(v);
            x(2) = 0;
            X[v.getIndex()] = x.normalized() + offset;
        }
    return X;
}

vecs tutte_embedding(const io::GeometryCentralMesh& mesh,vecs X) {
    auto V = mesh.mesh->nVertices();
    auto oldX = X;
    for (const auto& v : mesh.mesh->vertices()){
        if (!v.isBoundary()){
            vec x = vec::Zero();
            for (const auto& h : v.adjacentVertices())
                x += oldX[h.getIndex()];
            x /= v.degree();
            X[v.getIndex()] = x;
        }
    }
    return X;
}

Eigen::SimplicialLDLT<SMat> MCF;

void init_MCF(const io::GeometryCentralMesh& mesh,scalar dt = 1e-4){
    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexGalerkinMassMatrix();
    auto N = mesh.mesh->nVertices();
    SMat I(N,N);
    I.setIdentity();
    SMat L = geometry.vertexGalerkinMassMatrix - dt*(-geometry.cotanLaplacian + 1e-5*I);
    MCF.compute(L);
}

vecs mean_curvature_flow(const io::GeometryCentralMesh& mesh,const vecs& X) {
    return matToPos(MCF.solve(mesh.position_geometry->vertexGalerkinMassMatrix*posToMat(X)));
}

Eigen::SimplicialLDLT<SMat> TP,TN;

void init_taubin(const io::GeometryCentralMesh& mesh,scalar l = 1e-4,scalar mu = 2e-4){
    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireCotanLaplacian();
    geometry.requireVertexGalerkinMassMatrix();
    auto N = mesh.mesh->nVertices();
    SMat I(N,N);
    I.setIdentity();
    SMat LP = geometry.vertexGalerkinMassMatrix - l*(-geometry.cotanLaplacian + 1e-5*I);
    TP.compute(LP);
    SMat LN = geometry.vertexGalerkinMassMatrix + mu*(-geometry.cotanLaplacian + 1e-5*I);
    TN.compute(LN);
}

vecs taubin_step(const io::GeometryCentralMesh& mesh,const vecs& X,scalar l = 1e-1) {
    scalar mu = -0.9*l;
    auto P = posToMat(X);
    const auto& L = mesh.position_geometry->cotanLaplacian;
    P += l*L*P;
    P += mu*L*P;
    return matToPos(P);
}



Vec vertexDataToVec(const io::GeometryCentralMesh& M,const geometrycentral::surface::VertexData<double>& D){
    Vec X(M.mesh->nVertices());
    for (const auto& v : M.mesh->vertices())
        X(v.getIndex()) = D[v];
    return X;
}

Vec clampExtrems(const Vec& X) {
    int N = X.rows();
    scalars v(N);
    for (int i = 0;i<N;i++)
        v[i] = X(i);
    auto t = 0.01;
    int r = N*t;
    std::nth_element(v.begin(),v.begin() + r,v.end());
    auto m = *(v.begin() + r);
    int R = N*(1-t);
    std::nth_element(v.begin(),v.begin() + R,v.end());
    auto M = *(v.begin() + R);
    Vec Y = X;
    return Y.unaryExpr([m,M](scalar x) {return std::clamp<scalar>(x,m,M);});
}


Mat Curvatures(const io::GeometryCentralMesh& mesh) {
    Mat curvature;
    std::string cache = UPS_prefix + "cache/curvature.csv";
    if (io::MatrixCache(cache,curvature))
        return curvature;
    geometrycentral::surface::ExtrinsicGeometryInterface& geometry = *mesh.position_geometry;
    geometry.requireVertexMinPrincipalCurvatures();
    geometry.requireVertexMaxPrincipalCurvatures();
    geometry.requireVertexGaussianCurvatures();
    geometry.requireVertexMeanCurvatures();
    auto N = mesh.mesh->nVertices();
    mesh.mesh->vertices();
    int k = 4;
    curvature = Mat(N,k);
    curvature.col(0) = clampExtrems(vertexDataToVec(mesh,geometry.vertexMinPrincipalCurvatures));
    curvature.col(1) = clampExtrems(vertexDataToVec(mesh,geometry.vertexMaxPrincipalCurvatures));
    curvature.col(2) = clampExtrems(vertexDataToVec(mesh,geometry.vertexMeanCurvatures));
    curvature.col(3) = clampExtrems(vertexDataToVec(mesh,geometry.vertexGaussianCurvatures));
    io::SaveMatrix(cache,curvature);
    return curvature;
}

inline vec Vector3Tovec(const geometrycentral::Vector3& x) {
    return vec(x.x,x.y,x.z);
}

std::pair<scalars,scalars> computeDivAndCurl(const io::GeometryCentralMesh& mesh,const vecs& XF) {
    scalars div(mesh.mesh->nVertices(),0);
    scalars curl(mesh.mesh->nVertices(),0);
    mesh.position_geometry->requireVertexPositions();
    mesh.position_geometry->requireFaceNormals();
    for (const auto& v : mesh.mesh->vertices()){
        scalar d = 0;
        scalar c = 0;
        for (const auto& h : v.outgoingHalfedges()){
            auto u = XF[h.face().getIndex()];
            auto e = (mesh.position_geometry->vertexPositions[h.tipVertex()] - mesh.position_geometry->vertexPositions[h.next().tipVertex()]);
            d += u.dot(Vector3Tovec(geometrycentral::cross(mesh.position_geometry->faceNormal(h.face()),e)));
            c += u.dot(Vector3Tovec(e));
        }
        div[v.getIndex()] = d;
        curl[v.getIndex()] = c;
    }
    return {div,curl};
}

vecs generateFaceBasedRandomVectorField(const io::GeometryCentralMesh& mesh) {
    vecs XF(mesh.mesh->nFaces());
    for (const auto& f : mesh.mesh->faces()){
        XF[f.getIndex()] = vec::Random();
        XF[f.getIndex()](2) = 0;
    }
    //smooth vector field by iteratively avering with neighbors faces
    for (int i = 0;i<100;i++){
        vecs XF2(mesh.mesh->nFaces());
        for (const auto& f : mesh.mesh->faces()){
            vec x = vec::Zero();
            for (const auto& fn : f.adjacentFaces())
                x += XF[fn.getIndex()];
            XF2[f.getIndex()] = x/3.;
        }
        for (const auto& f : mesh.mesh->faces())
            XF[f.getIndex()] += 1e-1*XF2[f.getIndex()];
    }
    return XF;
}

Vec scalarsToVec(const scalars& S) {
    Vec X(S.size());
    for (int i =0;i<S.size();i++)
        X(i) = S[i];
    return X;
}

std::pair<vecs,vecs> computeHelmholtzHodge(const io::GeometryCentralMesh& mesh,const vecs& XF,const scalars& div) {
    mesh.position_geometry->requireCotanLaplacian();
    mesh.position_geometry->requireVertexGalerkinMassMatrix();
    mesh.position_geometry->requireFaceAreas();
    Vec D = scalarsToVec(div);
    SMat I(div.size(),div.size());
    I.setIdentity();
    SMat L = mesh.position_geometry->cotanLaplacian + 1e-5*I;
    auto P = geometrycentral::solvePositiveDefinite(L,(Vec)(mesh.position_geometry->vertexGalerkinMassMatrix*D));
    vecs DF(mesh.mesh->nFaces(),vec::Zero());
    vecs CF = DF;
    scalar S = 0;
    for (const auto& f : mesh.mesh->faces()){
        vec g = vec::Zero();
        auto N = mesh.position_geometry->faceNormal(f);
        for (const auto& h : f.adjacentHalfedges()){
            auto v = h.tailVertex();
            auto e = (mesh.position_geometry->vertexPositions[h.next().tipVertex()] - mesh.position_geometry->vertexPositions[h.tipVertex()]);
            g += P(v.getIndex())*Vector3Tovec(geometrycentral::cross(N,e));
        }
        g /= mesh.position_geometry->faceArea(f)*2;
        CF[f.getIndex()] = g;
        DF[f.getIndex()] = XF[f.getIndex()] - g;
        S += g.norm();
    }
    return {CF,DF};
}

io::GeometryCentralMesh maskGC;

void init () {

    const auto& SHOW = show;
    auto CMFONTID = UPS::FontManager::addFont(UPS_prefix + "fonts/ComputerModernSR.ttf",50);
    UPS::Style::default_font = CMFONTID;

    auto grid = Mesh::Add(UPS_prefix + "meshes/grid_quad_50.obj");
    auto disk = Mesh::Add(UPS_prefix + "meshes/disk_coarse.obj",0.5,true);
    auto bunny = Mesh::Add(UPS_prefix + "meshes/bunny.obj");
    auto bunny_coarse = Mesh::Add(UPS_prefix + "meshes/bunny_coarse.obj");
    auto human = Mesh::Add(UPS_prefix + "meshes/human.obj",0.2);
    auto mountain = Mesh::Add(UPS_prefix + "meshes/mountain.obj",0.1);
    maskGC.init(UPS_prefix + "meshes/nefertiti.obj");
    bunnyGC.init(UPS_prefix + "meshes/bunny.obj");
    polyscope::view::resetCameraToHomeView();

    auto arrow = Formula::Add("\\longrightarrow");

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
        auto plot = grid->apply(phi,false);
        show << inNextFrame << plot;//->at(0.5);
        auto explorer = Point::Add(point_explore,0.1)->apply(phi);
        show << explorer;
        explorer_id = explorer->pid;
        show << CameraView::Add(vec(0,-3,3.5),vec(0,0,1),vec::UnitZ());

        show << inNextFrame << disk->applyDynamic(taylor);
        show << inNextFrame;
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
        auto varphi = Formula::Add("\\varphi");
        auto mani = grid->apply(sphere_offset);
        mani->pc->setEdgeWidth(1);
        auto arrowp = arrow->at(0.5,0.65);
        show << mani;
        show << inNextFrame << grid_param << arrowp << PlaceBelow(varphi);

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

        show << PlaceRight(Formula::Add("\\varphi(x,y) = " + tex::Vec("sin(x)cos(y)","sin(x)sin(y)","cos(x)")),0.3,0.1);

        {
            StateInSlide t = arrowp.second;
            t.angle = M_PI;

            show << inNextFrame >> title << Title("Paramétrisation inverse ?")->at(TOP) << arrow->at(t);
            show << newFrame << Title("Vecteurs tangents")->at(TOP) << manifold << mani->at(0.7);
            auto P = Point::Add(vec(0,0,0));
            auto step = [] (scalar t) {
                return (1-smoothstep(0.3*t))*0.3+0.01;
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
                auto TM =  grid->applyDynamic([po](const Mesh::Vertex& v,TimeObject){
                    auto x = v.pos;
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
            show << "metric";
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
            show << AddPolyscopeQuantity(gplot);
            show << inNextFrame;
            for (int i = 0;i<6;i++){
                vec p = point_explore(polyscope::randomUnit()*M_PI*2)*2.5;
                vec q = point_explore(polyscope::randomUnit()*M_PI*2)*2.5;
                auto gamma = [q,p](scalar t){
                    t = periodic01(t*0.15)*15;
                    vec rslt = hyperbolic_geodesic(q,p,t-5.)*0.5;
                    return rslt;
                };
                show << Point::Add(gamma,0.02);
            }
            show << inNextFrame << PlaceRight(Image::Add(UPS_prefix + "images/pavage_penta.png"),0.6,0.01);
            show << newFrame << Title("Récap Géométrie différentielle 1")->at(TOP);
            show << inNextFrame << PlaceLeft(Latex::Add("Approche différentielle : approximation locale par des polynômes"),0.4);
            show << inNextFrame << PlaceRelative(Latex::Add("Paramétrisation : fonction d'exploration de la variété"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Espace tangent : plan tangent à un point de la variété"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Tenseur métrique : changement dans le calul des angles et longueur sur la surface"),ABS_LEFT,REL_BOTTOM,0.1);
        }
    }


    if (true)
    {
        show << newFrame << Title("Représentation discrète des formes et fonctions")->at(CENTER) << inNextFrame << TOP;
        scalar off = 3;
        auto bunny_off = bunny->apply(offset(vec(off,0,0)));
        show << bunny_off;
        auto bco =bunny_coarse;
        bco->pc->setEdgeWidth(1.);
        bco->pc->setSmoothShade(false);
        show << "rpz";
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
        auto Fq = AddPolyscopeQuantity(bpc->pc->addScalarQuantity("V0000",F));
        show <<  bpc << Fq;
        show << inNextFrame << PlaceRight(Formula::Add(tex::Vec("1.21","0.32", "\\vdots", "5.2","3.24"),0.05),0.6,0.1);
    }

    if (true)
    {
        show << newFrame << Title("Discrétisation des opérateurs différentiels")->at(TOP);
        show << inNextFrame << PlaceLeft(Latex::Add(tex::center("Opérateurs différentiels classiques")));
        show << PlaceBelow(Formula::Add("\\partial (f + g) = \\partial f + \\partial g")) << inNextFrame << PlaceRelative(Formula::Add(tex::AaboveB("?","\\longrightarrow"),0.06),UPS::CENTER_X,SAME_Y);
        show << inNextFrame << PlaceRight(Latex::Add(tex::center("Opérateurs différentiels discrets")));
        show << PlaceBelow(Formula::Add("AX"));

        show << newFrame << Title("Récap Discrétisation")->at(TOP);
        show << inNextFrame << PlaceLeft(Latex::Add("Représentation par maillage : information géométrique et topologique (graphe)"),0.4);
        show << inNextFrame << PlaceRelative(Latex::Add("Adapté à la mesure de données réelles"),ABS_LEFT,REL_BOTTOM,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("Fonction sur graphe $\\iff$ Vecteur"),ABS_LEFT,REL_BOTTOM,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("Opérateurs différentiels $\\iff$ Matrice"),ABS_LEFT,REL_BOTTOM,.1);
        //show << inNextFrame << PlaceRelative(Latex::Add("Tenseur métrique : changement dans le calul des angles et longueur sur la surface"),ABS_LEFT,REL_BOTTOM,0.1);
    }

    if (true)
    {
        using namespace tex;
        show << newFrame << Title("Exemple : Le laplacien $\\Delta$")->at(TOP);
        auto df = "\\partial^2 f";
        auto lapdef = Formula::Add("\\Delta f = " + frac(df,del<2>(0)) + "+" + frac(df,del<2>(1)),0.06);
        show << inNextFrame << lapdef->at(CENTER);
        auto topolap = Latex::Add(equation("(Lf)_i = \\sum_{n \\in N_i} (f_i - f_n)"),0.06);
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
            //auto Q = PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(diskverycoarse->pc->addVertexScalarQuantity("FEM",fem_basis));
            auto Q = AddPolyscopeQuantity(diskverycoarse->pc->addVertexScalarQuantity("FEM",fem_basis));
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


        {
            auto titlelap = Title("Applications du laplacien");
            show << newFrame << titlelap;
            mountain->setSmooth(true);
            show << inNextFrame << TOP << PlaceBelow(Latex::Add("Analyse de hauteur",0.05));
            auto fh = Formula::Add("f(x,y,z) = z",0.06);
            show << PlaceBelow(fh) <<mountain << top_cam;//CameraView::Add(vec(0,-3,3.5),vec(0,0,1),vec::UnitZ());
            auto H = mountain->eval([](const vec& x) {return x(1);});
            auto Q = AddPolyscopeQuantity(mountain->pc->addVertexScalarQuantity("H",H));
            auto LH = LapHeight();
            auto QL = AddPolyscopeQuantity(mountain->pc->addVertexScalarQuantity("LH",LH));
            QL->q->setColorMap("jet");
            show << Q << inNextFrame >> Q << QL << Formula::Add("\\Delta f",0.06)->at(0.8,0.4);

            {

                show << newFrameSameTitle  << PlaceBelow(Latex::Add("Energie de Dirichlet",0.05));
                show << inNextFrame << PlaceBelow(Formula::Add("E(f) = \\int_\\Omega ||\\nabla f||^2 dx",0.07),0.1);
            }


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
                auto title = Title("Focus sur le gradient : $\\nabla$");
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

                show << AddPolyscopeQuantity(fval);
                show << inNextFrame << PlaceRight(Formula::Add("\\nabla f(x,y) = "+ tex::Vec("x","y")),0.5);
                show << AddPolyscopeQuantity(gfval);

                auto N = 100;
                vecs GD(N);GD[0] = vec(1,1,0);
                for (int i = 1;i<N;i++)
                    GD[i] = GD[i-1] - 0.1*gradf(GD[i-1]);
                show << inNextFrame << PointCloud::Add(GD);
                Primitive::get<PointCloud>(show.lastPrimitiveInserted().first->pid)->pc->setPointRadius(0.02,false);
                show >> grad << Title("Descente de gradient")->at(TOP) << PlaceBelow(Formula::Add("x_{n+1} = x_{n} - \\tau \\nabla f(x_n)"));
            }

            {
                show << newFrame  << Title("Minimiser l'énergie de Dirichlet")->at(TOP);
                show << PlaceLeft(Formula::Add("E(f) = \\int_\\Omega ||\\nabla f||^2 dx",0.05),0.2,0.1);
                show << inNextFrame << PlaceRight(Latex::Add("On peut montrer que $\\nabla E(f) = \\Delta f$",0.05),0.2,0.1);
                show << inNextFrame << PlaceBelow(Formula::Add("f_{n+1} = f_n - \\tau \\Delta f_n",0.05));
                auto off = vec(1.5,0,0);
                auto mask = Mesh::Add(UPS_prefix + "meshes/nefertiti.obj",0.5)->apply(offset(off),false);
                show << mask << top_cam;
                auto TI = tutte_init(maskGC,-off);
                Mesh::MeshPtr te_mesh = Mesh::Add(TI,mask->getFaces(),false);
                te_mesh->updater = [TI] (const TimeObject& t,Primitive* ptr){
                    static int cid = -1;
                    Mesh* p = static_cast<Mesh*>(ptr);
                    if (t.relative_frame_number == 0){
                        if (cid != 0)
                            p->updateMesh(TI);
                        cid = 0;
                    }
                    else {
                        cid = 1;
                        p->updateMesh(tutte_embedding(maskGC,p->getVertices()));
                    }
                };
                show << te_mesh << inNextFrame;
            }



            {
                show << newFrame << titlelap->at(TOP);
                show << PlaceBelow(Latex::Add("Fonctions harmoniques",0.05));
                show << inNextFrame << PlaceBelow(Formula::Add("\\Delta f = 0",0.05),0.05);
                show << PlaceLeft(Image::Add(UPS_prefix + "images/poisson-g.png"),0.5,0.1);
                show << inNextFrame << PlaceRelative(Image::Add(UPS_prefix + "images/poisson-u.png"),REL_RIGHT,SAME_Y,0.1);
                show << newFrameSameTitle << PlaceBelow(Latex::Add("Problème de Plateau (ou du film de savon)",0.04));
                show << PlaceBelow(Image::Add(UPS_prefix + "images/plateau.png"));
                show << newFrameSameTitle << PlaceBelow(Latex::Add("Poisson Surface Reconstruction",0.04));
                show << PlaceBelow(Image::Add(UPS_prefix + "images/PSR.jpg"));
                show << inNextFrame << PlaceBelow(Image::Add(UPS_prefix + "images/PSR_algo.png"));
            }

            if (false)
            {
                auto cam = CameraView::Add(vec(-0.5,2,9),vec(-0.5,2,0),vec::UnitY());
                show << newFrameSameTitle << PlaceBelow(Latex::Add("Problème de Poisson",0.05));
                show << PlaceBelow(Formula::Add("\\Delta u = y",0.06),0.1);
                show << PlaceLeft(Formula::Add("y",0.05),0.35,0.3);
                UPS::Mat C = illustratePoissonProblem();
                auto off = vec(2.5,0,0);
                auto bunnyY = DuplicatePrimitive(bunny_coarse)->apply(offset(-off));
                bunnyY->pc->setSmoothShade(false);
                bunnyY->pc->setEdgeWidth(1.);
                auto bunnyX = DuplicatePrimitive(bunny_coarse)->apply(offset(off));
                bunnyX->pc->setSmoothShade(false);
                bunnyX->pc->setEdgeWidth(1.);
                auto QY = AddPolyscopeQuantity(bunnyY->pc->addVertexScalarQuantity("Y",C.col(0)));
                auto QX = AddPolyscopeQuantity(bunnyX->pc->addVertexScalarQuantity("X",C.col(1)));
                show << cam << bunnyY << QY;
                show << inNextFrame << Formula::Add(tex::AaboveB("\\Delta^{-1}","\\longrightarrow"),0.06);
                show << bunnyX << QX;
                show << PlaceRight(Formula::Add("u",0.05),0.35,0.3);
            }


            {
                show << newFrame << titlelap->at(TOP) << PlaceBelow(Latex::Add("Décomposition spectrale",0.05));
                show << PlaceBelow(Formula::Add("\\Delta u = \\lambda u",0.07),0.05);
                show << inNextFrame <<PlaceBelow(Formula::Add("\\Delta (e^{ix}) = -e^{ix}",0.05));
                auto cam2 = CameraView::Add(vec(-0.5,6,15),vec(-0.5,6,0),vec::UnitY());
                show << inNextFrame << cam2;
                auto humanGC = io::GeometryCentralMesh(UPS_prefix + "meshes/human.obj");
                auto BCGC = io::GeometryCentralMesh(UPS_prefix + "meshes/bunny_coarse.obj");
                auto eig_h = EigenLaplace(humanGC,"human",10);
                auto eig_b = EigenLaplace(BCGC,"bunny",300);
                auto NB_slider = AddIntSliders(1,"NB of coeffs",eig_b.cols(),{1,eig_b.cols()});


                for (int i = 0;i<10;i++){
                    Mesh::MeshPtr human_eig;
                    if (i < 5)
                        human_eig = human->apply(offset(vec(-6 + 3*i,3,0)));
                    else
                        human_eig = human->apply(offset(vec(-6 + 3*(i-5) + 1.5,0,0)));
                    human_eig->setSmooth(true);
                    auto E = AddPolyscopeQuantity(human_eig->pc->addVertexScalarQuantity("eig",eig_h.col(i)));
                    show << human_eig << E;
                }

                show << newFrameSameTitle;
                show << NB_slider;
                auto bunnySC = DuplicatePrimitive(bunny_coarse);
                show << bunnySC << CameraView::Add(vec(-0.5,2,8),vec(-0.5,2,0),vec::UnitY());
                vecs L = compute_eigen_coeffs(eig_b,bunnySC->getVertices());
                bunnySC->updater = [eig_b,L,NB_slider] (const TimeObject& t,Primitive* ptr){
                    Mesh* p = static_cast<Mesh*>(ptr);
                    static int cid = -1;
                    int N = NB_slider->getVal(0);
                    if (cid != N){
                        p->updateMesh(spectral_compression(eig_b,L,NB_slider->getVal(0)));
                        cid = N;
                    }
                };
            }

            show << newFrame << Title("Récap Laplacien")->at(TOP);
            show << inNextFrame << PlaceLeft(Latex::Add("Compare la valeur d'une fonction en un point par rapport au voisinage"),0.4);
            show << inNextFrame << PlaceRelative(Latex::Add("Interpolation"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Diffusion de quantités"),ABS_LEFT,REL_BOTTOM,0.1);
            show << inNextFrame << PlaceRelative(Latex::Add("Analyse spectrale"),ABS_LEFT,REL_BOTTOM,0.1);

        }

    }

    if (false)
    {
        auto parabola = [] (const Mesh::Vertex& v,const TimeObject& t) {
            const auto& h = v.pos;
            if (t.relative_frame_number == 0)
                return h;
            scalar a = -0.3,b = 0.4;
            if (t.relative_frame_number == 1){
                a *= smoothstep(t.from_action*0.3);
                b *= smoothstep(t.from_action*0.3);
            }
            return vec(h(0),h(1),a*h(0)*h(0) + b*h(1)*h(1));
        };
        show << newFrame << Title("Courbures");
        show << "curvature";
        auto plot = grid->applyDynamic(parabola,false);
        show << inNextFrame << TOP << plot << inNextFrame;
        show << inNextFrame << Formula::Add("k_1,k_2",0.06)->at(0.5,0.3);
        show << newFrameSameTitle;
        auto macaca = Mesh::Add(UPS_prefix + "meshes/david.obj",0.01);
        io::GeometryCentralMesh mesh(UPS_prefix + "meshes/david.obj");
        Mat K = Curvatures(mesh);

        auto LM = [] (scalar x) {
            return std::log(1+std::abs(x))*sgn(x);
        };

        for (int i = 0;i<K.cols();i++){
            auto macaca_K = macaca->apply(offset(vec(-3 + 2*i,0,0)),true);
            auto q = macaca_K->pc->addVertexScalarQuantity("curvature",K.col(i));//.unaryExpr(LM));
            auto E = AddPolyscopeQuantity(q);
            q->setColorMap("jet");
            show << macaca_K << E;
        }
        show << CameraView::Add(vec(0.5,-6,1.5),vec(0.5,0,1.5),vec::UnitZ());
        show << Formula::Add("k_1",0.05)->at(0.2,0.3);
        show << Formula::Add("k_2",0.05)->at(0.4,0.3);
        show << Formula::Add("H = " + tex::frac("k_1 + k_2","2"),0.05)->at(0.6,0.3);
        show << Formula::Add("K = k_1k_2",0.05)->at(0.8,0.3);

        show << newFrame << Title("Discrétisation courbure de Gauss")->at(TOP);
        auto deflect = PlaceRight(Formula::Add("K = 2\\pi - \\sum_{n \\in N_i} \\theta_n",0.06),0.4,0.1);
        auto fan = Mesh::Add(UPS_prefix + "meshes/fan.obj");
        auto curvature = [] (const Mesh::Vertex& v,const TimeObject& t) {
            const auto& h = v.pos;
            if (t.relative_frame_number == 0)
                return h;
            scalar a = 0.4,b = 0.4;
            if (t.relative_frame_number == 1){
                a *= smoothstep(t.from_action*0.5);
                b *= smoothstep(t.from_action*0.5);
            }
            else {
                a *= 1-2*smoothstep(t.from_action*0.5);
            }
            return vec(h(0),h(1),a*h(0)*h(0) + b*h(1)*h(1));
        };
        show << fan->applyDynamic(curvature,false);
        show << deflect;
        auto S = show.getCurrentSlide();
        show << PlaceRight(Formula::Add("K = 0",0.05),0.6);
        show << S << PlaceRight(Formula::Add("K = 1",0.05),0.6);
        show << S << PlaceRight(Formula::Add("K = -1",0.05),0.6);

        show << newFrame;
        show << Title("Théorème de Gauss-Bonet")->at(TOP);
        show << inNextFrame << Formula::Add("\\int_{\\mathcal{M}} K dA = 4(1-g)",0.08);

        show << newFrame << Title("Discrétisation courbure moyenne")->at(TOP);
        show << PlaceBelow(Formula::Add("\\Delta P = -2Hn",0.06));

        init_MCF(bunnyGC,1e-3);
        init_taubin(bunnyGC,1e-3,2e-3);

        auto X0 = bunny->getVertices();
        auto bunnymcf = DuplicatePrimitive(bunny);
        bunnymcf->setSmooth(true);

        vecs noise(bunnyGC.mesh->nVertices());
        for (auto& x : noise)
            x = vec::Random()*0.01;

        bunnymcf->updater = [X0,noise] (const TimeObject& t,Primitive* ptr){
            static int cid = -1;
            Mesh* p = static_cast<Mesh*>(ptr);
            if (t.relative_frame_number == 0){
                if (cid != 0)
                    p->updateMesh(X0);
                cid = 0;
            }
            else if (t.relative_frame_number == 1){
                p->updateMesh(mean_curvature_flow(bunnyGC,p->getVertices()));
                cid = 1;
            }
            else if (t.relative_frame_number == 2){
                auto Xt = X0;
                for (int i = 0;i<Xt.size();i++)
                    Xt[i] += noise[i]*smoothstep(t.from_action);
                p->updateMesh(Xt);
                cid = 1;
            }
            else {
                cid = 1;
                //p->updateMesh(mean_curvature_flow(bunnyGC,p->getVertices()));
                p->updateMesh(taubin_step(bunnyGC,p->getVertices()));
            }
        };
        show << CameraView::Add(vec(-0.5,2,9),vec(-0.5,2,0),vec::UnitY());
        show << bunnymcf;
        show << inNextFrame <<PlaceRight(Latex::Add("Lissage : Mean Curvature Flow"),0.3);
        show << inNextFrame <<PlaceBelow(Latex::Add("Débruitage : Taubin flow"));
        show << inNextFrame;

        show << newFrame << Title("Récap courbures")->at(TOP);
        show << inNextFrame << PlaceLeft(Latex::Add("Mesure l'échec d'une surface à ressembler à un plan"),0.4);
        show << inNextFrame << PlaceRelative(Latex::Add("Différents types de courbures : Min/Max, Moyenne, Gauss"),ABS_LEFT,REL_BOTTOM,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("Anaylse de surface et débruitage"),ABS_LEFT,REL_BOTTOM,0.1);
    }

    if (false)
    {
        auto t = Title("Opérateurs vectoriels");
        show << newFrame << t;
        show << inNextFrame << TOP;
        auto diskGC = io::GeometryCentralMesh(UPS_prefix + "meshes/disk_coarse.obj");
        auto disk = Mesh::Add(UPS_prefix + "meshes/disk_coarse.obj",1.,true);
        auto XF = generateFaceBasedRandomVectorField(diskGC);
        auto Q = disk->pc->addFaceVectorQuantity("X",XF);
        auto top_cam2 = CameraView::Add(vec(0,0.6,2.5),vec(0,0.6,0),vec(0,1,0));
        show << disk << top_cam2 << AddPolyscopeQuantity(Q);
        auto&& [div,curl] = computeDivAndCurl(diskGC,XF);
        auto Qdiv = disk->pc->addVertexScalarQuantity("div",div);
        auto div_t = Formula::Add("\\text{div}(V)_i = \\sum_{ijk\\sim i}" + tex::dot("u_{ijk}","n_{ijk}\\times e_{jk}"),0.04);
        Qdiv->setColorMap("jet");
        auto S = show.getCurrentSlide();

        auto cam2 = CameraView::Add(vec(1,0.5,2),vec(1,0.5,0),vec(0,1,0),true);
        show << S << cam2 << Latex::Add("Divergence",0.05)->at(0.5,0.2) << AddPolyscopeQuantity(Qdiv) << PlaceRight(div_t,0.4,0.05) << PlaceBelow(Image::Add(UPS_prefix + "images/fan_div.png"));
        auto Qcurl = disk->pc->addVertexScalarQuantity("curl",curl);
        auto curl_t = Formula::Add("\\text{curl}(V)_i = \\sum_{ijk\\sim i}" + tex::dot("u_{ijk}","e_{jk}"),0.04);
        Qcurl->setColorMap("jet");
        show << S << cam2 << Latex::Add("Rotationnel",0.05)->at(0.5,0.2) << AddPolyscopeQuantity(Qcurl) << PlaceRight(curl_t,0.4,0.05) << PlaceBelow(Image::Add(UPS_prefix + "images/fan_curl.png"));
        show << newFrameSameTitle << PlaceBelow(Latex::Add("Théorème de Helmotz-Hodge",0.05));
        auto HH = computeHelmholtzHodge(diskGC,XF,div);
        auto disk_orig = disk->apply(offset(vec(-2.1,0,0)));
        auto disk_CF = disk->apply(offset(vec(0,0,0)));
        auto disk_DF = disk->apply(offset(vec(2.1,0,0)));
        auto orig = disk_orig->pc->addFaceVectorQuantity("X",XF);
        auto QCF = disk_CF->pc->addFaceVectorQuantity("CF",HH.first);
        auto QDF = disk_DF->pc->addFaceVectorQuantity("DF",HH.second);
        show << top_cam << disk_orig << disk_CF << disk_DF << AddPolyscopeQuantity(orig) <<  AddPolyscopeQuantity(QCF) << AddPolyscopeQuantity(QDF);
        show << Formula::Add("=",0.06)->at(1./3.,0.5) << Formula::Add("+",0.06)->at(2./3.,0.5);
        show << Formula::Add("X",0.06)->at(1./6.,0.9) << Formula::Add("\\text{curl}(X) = 0",0.06)->at(1./2.,0.9) << Formula::Add("\\text{div}(X) = 0",0.06)->at(5./6.,0.9);
        show << newFrame << Title("Laplacien vectoriel et diffusion");
    }

    if (true) {
        show << newFrame << Title("Références") << TOP;
        show << inNextFrame << PlaceLeft(Latex::Add("\"Introduction to discrete differential geometry\", Keenan Crane (Cours filmé)"),0.4);
        show << inNextFrame << PlaceRelative(Latex::Add("\"Polygon Mesh Processing\", Bruno Lévy et at."),ABS_LEFT,REL_BOTTOM,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("\"Analyse Numérique et optimisation\", Grégoire Allaire"),ABS_LEFT,REL_BOTTOM,0.1);
    }


}



int main(int argc,char** argv) {
    show.init(UPS_prefix + "../../projects/course_MG/script.txt");
    //show.init();
    init();

    polyscope::state::userCallback = [](){
        show.play();
        //ImGui::ShowDemoWindow();
    };
    polyscope::show();
    return 0;
}
