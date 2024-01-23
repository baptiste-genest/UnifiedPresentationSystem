#include "polyscope/polyscope.h"
#include "../../src/UnifiedPresentationSystem.h"

using namespace UPS;

Slideshow show;

vecs randomPointCloud(int N) {
    vecs points(N);
    for (int i = 0;i<N;++i) {
        points[i] = vec::Random()*0.5;
        points[i](1) = 0;
    }
    return points;
}

vecs project(vecs X,vec s) {
    for (auto& x : X)
        x = vec(s.dot(x)*s);
    return X;
}

curve_param roundArrowParam(const vec& a ,const vec& b) {
    vec mid = (a+b)/2;
    vec dx = b-mid;
    int side = rand()%2;
    return [mid,dx,side] (scalar t) {
        if (side)
            return vec(mid + Eigen::AngleAxis<scalar>(t*M_PI,vec(0,1,0))*dx);
        return vec(mid + Eigen::AngleAxis<scalar>(-t*M_PI,vec(0,1,0))*dx);
    };
}

vec proj(const vec& x,const vec& s) {
    return vec(s.dot(x)*s);
}

using labeled_cloud = std::vector<std::pair<vec,int>> ;

labeled_cloud labelize(const vecs& X) {
    labeled_cloud rslt(X.size());
    for (int i = 0;i<X.size();i++)
        rslt[i] = {X[i],i};
    return rslt;
}

vecs subsample(const vecs& X,int N) {
    vecs rslt(N);
    for (int i = 0;i<N;i++)
        rslt[i] = X[rand()%X.size()];
    return rslt;
}

using VFPC = std::shared_ptr<PolyscopeQuantity<polyscope::PointCloudVectorQuantity>>;
bool runall = false;
void init() {

    srand(time(NULL));

    Latex::NewCommand("W","\\mathcal{W}");
    Latex::NewCommand("U","\\mathcal{U}");
    Latex::NewCommand("Sp","\\mathbb{S}");
    Latex::NewCommand("proj","\\Pi^\\theta");

    int N = 20;

    if (false || runall)
    {
        show << Title("Non Euclidean Sliced \\\\ Optimal Transport Sampling")->at(UPS::CENTER);
        show << PlaceBelow(Latex::Add(tex::center("Baptiste Genest \\\\ David Coeurjolly \\\\ Nicolas Courty"),Options::UPS_default_height_ratio*0.8),0.1);
        show << PlaceBelow(Latex::Add("Eurographics 2024",Options::UPS_default_height_ratio*0.7),0.1);

        show << newFrame << Title("Optimal Transport")->at(UPS::TOP);
        show << PlaceBelow(Latex::Add("Discrete to Discrete"));
        show << Image::Add("OT_1.png");
        show << inNextFrame << Replace(Image::Add("OT_2.png"));

        show << newFrame << Title("How hard is it?")->at(UPS::TOP);
        show << inNextFrame << PlaceBelow(Latex::Add("Exact discrete to discrete : Linear Programming $\\implies \\mathcal{O}(n^3)$"),0.1);
        show << inNextFrame << PlaceBelow(Latex::Add("Trivial in 1 case : 1D-OT"),0.1);
        show << inNextFrame << PlaceBelow(Image::Add("OT_1D.png",0.7),0.05);
        show << inNextFrame << PlaceBelow(Latex::Add("$\\mathcal{O}(n\\log(n))$!",Options::UPS_default_height_ratio*1.7));
    }

    if (false || runall)
    {
        show << newFrame << Title("Sliced Optimal Transport")->at(UPS::TOP);

        show << inNextFrame << Formula::Add("\\W(\\mu,\\nu)")->at("W2");
        show << inNextFrame << PlaceNextTo(Formula::Add("\\approx SW(\\mu,\\nu) = \\int_{\\Sp} \\W(\\proj_\\# \\mu ,\\proj_\\# \\nu) d\\theta"),1);

        auto mu = randomPointCloud(N);
        auto mupc = PointCloud::Add(mu);
        auto nu = randomPointCloud(N);
        auto nupc = PointCloud::Add(nu);

        show << inNextFrame << mupc << nupc;
        show << CameraView::Add(Options::ProjectViewsPath + "planar.json");

        show << inNextFrame;
        for (int i = 0;i<2;i++) {
            vec theta = vec::Random();theta(1) = 0;theta.normalize();
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.002);
            auto slicing = [theta] (Vertex v,const TimeObject t) {
                if (t.relative_frame_number)
                    return proj(v.pos,theta);
                scalar f = std::clamp(t.from_action*0.6f,0.f,1.f);
                return vec(v.pos*(1-f) + proj(v.pos,theta)*f);
            };
            auto m = mupc->applyDynamic(slicing);
            auto n = nupc->applyDynamic(slicing);


            show <<  pct << inNextFrame << m << n;

            auto mup =mu;auto nup = nu;
            std::sort(mup.begin(),mup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});
            std::sort(nup.begin(),nup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});

            PrimitiveGroup Assigments;
            for (int i = 0;i<N;i++){
                Assigments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta)),100,false,0.002);
            }
            show <<inNextFrame << Assigments;
            show << inNextFrame >> pct >> m >> n >> Assigments;
        }
    }

    if (false || runall)
    {

        auto uniform = Image::Add("uniform.png");
        auto blue_noise = Image::Add("blue_noise.png");

        show << newFrame << Title("Sampling problem")->at(UPS::TOP);
        Panel noises;
        noises << Image::Add("uniform.png");
        noises << PlaceNextTo(Image::Add("blue_noise.png"),1);
        show << inNextFrame << noises;
        show << inNextFrame << PlaceRelative(Formula::Add("\\W(\\mu_1,\\U) > \\W(\\mu_2,\\U)"),CENTER_X,placeY::REL_BOTTOM,0,0.1);

    }

    if (false || runall)
    {
        show << newFrame << Title("Sliced Optimal Transport Sampling")->at(TOP);
        show << inNextFrame << PlaceBelow(Latex::Add("Stochastic Gradient Descent on $\\mu \\mapsto SW(\\mu,\\nu)$"),0.04);
        auto back = show.getCurrentSlide();
        show << CameraView::Add(Options::ProjectViewsPath + "planar_close.json");

        {
            auto swgrad = Formula::Add("\\nabla_{x_i} SW^\\theta ?");
            vec theta = vec(1,0,0);
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.002);

            auto mu = project(randomPointCloud(8),theta);
            auto mupc = PointCloud::Add(mu);
            auto nu = project(randomPointCloud(8),theta);
            auto nupc = PointCloud::Add(nu);

            show << inNextFrame << PlaceBelow(swgrad) <<  pct << mupc << nupc;

            auto mup =mu;auto nup = nu;
            std::sort(mup.begin(),mup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});
            std::sort(nup.begin(),nup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});

            PrimitiveGroup Assignments;
            for (int i = 0;i<N;i++){
                Assignments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta)),100,false,0.002);
            }
            show << Assignments;
            show << inNextFrame << Replace(Formula::Add("\\nabla_{x_i} SW^{\\theta} = T^{\\theta}(x_i) - x_i"),swgrad);
        }

        show << back << CameraView::Add(Options::ProjectViewsPath + "planar.json");
        auto mu = randomPointCloud(N);
        auto mupc = PointCloud::Add(mu);
        auto NU = randomPointCloud(N*20);
        auto NUpc = PointCloud::Add(NU);

        show << inNextFrame << mupc << NUpc;

        show << inNextFrame << NUpc->at(0.5) << inNextFrame;

        int K = 8;

        std::vector<vecs> gradients;

        std::vector<VFPC> grads_pc(K);

        for (int i = 0;i<K;i++) {

            auto nu = subsample(NU,N);
            auto nupc = PointCloud::Add(nu);

            show << nupc;

            vec theta = vec::Random();theta(1) = 0;theta.normalize();
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.002);
            auto slicing = [theta] (Vertex v,const TimeObject t) {
                if (t.relative_frame_number)
                    return proj(v.pos,theta);
                scalar f = std::clamp(t.from_action*0.6f,0.f,1.f);
                return vec(v.pos*(1-f) + proj(v.pos,theta)*f);
            };
            auto m = mupc->applyDynamic(slicing);
            auto n = nupc->applyDynamic(slicing);


            show <<  pct << inNextFrame << m << n;

            auto mup = labelize(mu);auto nup = labelize(nu);
            auto cmp = [theta](const std::pair<vec,int>& x,const std::pair<vec,int>& y) {return x.first.dot(theta) < y.first.dot(theta);};
            std::sort(mup.begin(),mup.end(),cmp);
            std::sort(nup.begin(),nup.end(),cmp);

            if (i == 0) {
                PrimitiveGroup Assigments;
                for (int i = 0;i<N;i++){
                    Assigments << Curve3D::Add(roundArrowParam(proj(mup[i].first,theta),proj(nup[i].first,theta)),100,false,0.002);
                }

                show << inNextFrame << Assigments;
                show << inNextFrame >> Assigments;
            }

            gradients.push_back(vecs(N));
            double max_length = 0;
            for (int j = 0;j<N;j++){
                gradients[i][nup[j].second] = proj(nup[j].first - mup[j].first,theta);
                max_length = std::min(max_length,-gradients[i][nup[j].second].norm());
            }

            auto gradpc = mupc->pc->addVectorQuantity("gradient" + std::to_string(i),gradients[i]);
            gradpc->setVectorLengthScale(-max_length*0.2,false);
            grads_pc[i] = AddPolyscopeQuantity<polyscope::PointCloudVectorQuantity>(gradpc);
            show << inNextFrame << grads_pc[i];

            show << inNextFrame >> pct >> m >> n >> nupc;
        }

        show << PlaceLeft(Formula::Add("\\nabla_{x_i} SW\\leftarrow  \\frac{1}{\\Theta}\\sum_{\\theta}\\nabla_{x_i} SW^\\theta"),0.5,0.03);

        vecs combined_gradients(N,vec::Zero());
        show << inNextFrame;
        for (int j = 0;j<K;j++){
            for (int i = 0;i<N;i++){
                combined_gradients[i] += gradients[j][i]/K;
            }
            show >> grads_pc[j];

        }
        for (int i = 0;i<N;i++)
            mu[i] += combined_gradients[i]*0.4;

        auto gradpc = mupc->pc->addVectorQuantity("combined gradient",combined_gradients);
        show << AddPolyscopeQuantity<polyscope::PointCloudVectorQuantity>(gradpc);
        show << inNextFrame << PointCloud::Add(mu) >> mupc;
    }
    auto grid = Mesh::Add(Options::DataPath+"meshes/tri_grid_50.obj");

    if (true || runall)
    {
        auto offset = 1.5;
        show << newFrame << Title("Non-Euclidean Sliced \\\\ Optimal Transport Sampling")->at(TOP);
        show << inNextFrame << Mesh::Add(Options::DataPath + "meshes/ico_sphere_5.obj")->translate(vec(-offset,0,0));
        show << CameraView::Add(Options::ProjectViewsPath + "models.json");
        show << grid->apply([offset](const vec& x) {
            auto u = x(0)*1.5,v = 2*M_PI*x(2);
            return vec(sinh(u)*cos(v) + offset,sinh(u)*sin(v),cosh(u)-1.);
        });
        show << Formula::Add("\\mathbb{S}^2")->at("S2");
        show << Formula::Add("\\mathbb{H}^2")->at("H2");
        show << newFrame << Title("Contributions")->at(TOP);
        show << Image::Add("sphere_sampling.png",0.6)->at("sphere_sampling");
        show << Image::Add("geomed_gain.png",0.6)->at("geomed");
        show << Image::Add("duck.png")->at("duck");
        show << Image::Add("rot_sampling.png",0.6)->at("rot");
        show << PlaceLeft(Latex::Add(tex::enumerate(
            "Extension to non-euclidean cases",
            "Use of the geometric median",
            "Intrinsic Mesh sampling",
            "Projective plane sampling")),0.5,0.05);
        show << newFrame << Title("NESOTS algorithm")->at(TOP);
        show << Image::Add("nesots_algo.png",0.8)->at("nesots");
        show << Image::Add("logexp.png",1)->at("logexp");
    }
    if (true || runall) {
        show << newFrame << Title("Geometric Median for robust SGD")->at(TOP);
        show << Latex::Add("Classic SGD:")->at("SGD");
        show << PlaceBelow(Formula::Add("\\nabla_{x_i} \\mathcal{SW}(\\sum_i \\delta_{x_i},\\nu) \\leftarrow \\frac{1}{L}\\sum_{l = 1}^{L} \\left( T(x_i) - x_i\\right) "));
        show << inNextFrame;
        show << Latex::Add("GeoMed Descent:")->at("GMD");
        show << PlaceBelow(Formula::Add("\\text{GeoMed}(\\{ T(x_i) - x_i\\}_i)"));
        show << inNextFrame;
        show << Image::Add("geomed_gain.png")->at("geomed2");
    }
    if (true || runall) {
        show << newFrame << Title("Intrinsic Mesh Sampling")->at(TOP);
        show << Image::Add("mesh_sampling.png");
        show << inNextFrame << Replace(Image::Add("mesh_sampling_cmp.png"));
    }
    if (true || runall) {
        show << newFrame << Title("Projective Plane Sampling")->at(TOP);
        show << PlaceLeft(Latex::Add("Rotation sampling $\\rightarrow$ sampling unit Quaternions $\\subset \\mathbb{S}^3$"),0.25);
        show << inNextFrame << Formula::Add("q^{-1}\\Vec{x}q")->at("quat");
        show << inNextFrame << PlaceNextTo(Formula::Add("=(-q)^{-1}\\Vec{x}(-q)"),1);
        show << inNextFrame << Image::Add("rot_sampling.png")->at("rot2");
        show << PlaceLeft(Latex::Add("Direction Sampling \\& more"),0.5);
        show << inNextFrame << Image::Add("dir_sampling.png",0.8)->at("dir");
        show << inNextFrame << Image::Add("line_sampling.png",0.8)->at("line");
    }

    show << newFrame << Title("Thank you for your attention");
    show << PlaceBelow(Latex::Add("Code Available : Eulerson314 (Github)"));


}

int main(int argc,char** argv)
{
    show.init("NESOTS");
    if (argc > 1)
        runall = true;

    init();

    polyscope::state::userCallback = [](){
        show.play();
    };
    polyscope::show();
    return 0;
}
