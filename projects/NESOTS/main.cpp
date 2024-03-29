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

vecs random1DPointCloud(int N,double bias = 0) {
    vecs points(N);
    for (int i = 0;i<N;++i) {
        points[i] = vec::Zero();
        points[i](0) = polyscope::randomReal(-0.5,0.5) + bias;
    }
    return points;
}

vecs readPointCloud(const std::string& path) {
    std::ifstream file(path);
    vecs points;
    while (file){
        vec point;
        file >> point(0) >> point(1) >> point(2);
        points.push_back(point);
    }
    return points;
}

std::vector<int> readPlan(const std::string& path,int N) {
    std::ifstream file(path);
    std::vector<int> plan(N);
    for (int i = 0;i<N;++i) {
        file >> plan[i];
    }
    return plan;
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

scalar angle(const vec& a,const vec& b){
    return std::acos(a.dot(b));
}

vec slerp(float t,const vec& a,const vec& b){
    auto th = angle(a,b);
    return (std::sin((1-t)*th)*a + std::sin(t*th)*b)/std::sin(th);
}

vec Log(const vec& x,const vec& y){
    auto d = angle(x,y);
    vec delta = y-x;
    delta -= delta.dot(x)*x;
    return d*delta.normalized();
}

std::function<vec2()> trackScreen(const std::function<vec()>& pos) {
    return [pos] () {
        auto p = pos();
        glm::vec4 pos = glm::vec4(p(0),p(1),p(2),1);
        glm::vec4 screenPos = polyscope::view::getCameraPerspectiveMatrix()*polyscope::view::viewMat * pos;
        screenPos /= screenPos.w;
        screenPos = (screenPos + glm::vec4(1,1,1,1))/2.f;
        screenPos.y = 1-screenPos.y;
        return vec2(screenPos.x,screenPos.y);
    };
}

std::shared_ptr<CurveNetwork> plotPlan(const vecs& mu,const vecs& nu,const std::vector<int>& T) {
    vecs X = mu;
    Curve3D::edges E(mu.size());
    X.insert(X.end(),nu.begin(),nu.end());
    for (int i = 0;i<mu.size();i++){
        E[i] = {i,T[i] + (int)mu.size()};
    }
    return CurveNetwork::Add(X,E);
}

vec orthoproj(vec a,const vec& n) {
    a -= a.dot(n)*n;
    return a;
}

void GramSchmidt(const vec& x,vec& a,vec& b) {
    a -= x.dot(a)*x;
    a.normalize();
    b -= x.dot(b)*x;
    b -= a.dot(b)*a;
    b.normalize();
}

using VFPC = std::shared_ptr<PolyscopeQuantity<polyscope::PointCloudVectorQuantity>>;
bool runall = false;
void init() {

    srand(time(NULL));

    Latex::NewCommand("W","\\mathcal{W}");
    Latex::NewCommand("U","\\mathcal{U}");
    Latex::NewCommand("Sp","\\mathbb{S}");
    Latex::NewCommand("proj","\\Pi^\\theta");

    Options::UPS_screen_resolution_x = 1920;
    Options::UPS_screen_resolution_y = 1080;

    int N = 20;

    {
        show << Title("Non Euclidean Sliced \\\\ Optimal Transport Sampling")->at(UPS::CENTER);
        show << PlaceBelow(Latex::Add(tex::center("Baptiste Genest \\\\  Nicolas Courty \\\\ David Coeurjolly"),Options::UPS_default_height_ratio*0.8),0.1);
        show << PlaceBelow(Latex::Add("Eurographics 2024",Options::UPS_default_height_ratio*0.7),0.1);

    }

    {
        show << newFrame << Title("Optimal Transport")->at(UPS::TOP);
        show << PlaceBelow(Latex::Add("Discrete to Discrete"));
        show << PlaceBottom(Latex::Add(tex::center("Among all bijections $\\gamma$, \\\\ which one minimizes the effort \\\\ to move each $x_i$ to $\\gamma(x_i)$?")),0.5,0.1);

        show << Image::Add("OT_1.png");
        show << inNextFrame << Replace(Image::Add("OT_2.png"));

    }
    {
        show << newFrame << Title("Optimal Transport")->at(UPS::TOP);
        show << PlaceBelow(Latex::Add("The Wasserstein distance"));
        show << Latex::Add("Given two probability measures $\\mu$ and $\\nu$")->at("wass_def");
        show << inNextFrame << Formula::Add(R"(\W_p^p(\mu,\nu) = \inf_{\gamma \in \Pi(\mu,\nu)} \int d(x,y)^p d\gamma)");
        show << inNextFrame << Latex::Add("Very importantly, $\\mu$ and $\\nu$ can be discrete or continuous!")->at("discrete");
    }
    {
        show << newFrame << Title("How hard is it?")->at(UPS::TOP);
        show << inNextFrame << PlaceBelow(Latex::Add("Exact discrete to discrete : Linear Programming $\\implies \\mathcal{O}(n^3)$"),0.1);
        show << inNextFrame << PlaceBelow(Latex::Add("Trivial in 1 case : 1D-OT"),0.1);
        show << CameraView::Add("1DOT");
        {

            vec theta = vec(1,0,0);
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.005);

            auto mu = random1DPointCloud(8,-0.1);
            auto mupc = PointCloud::Add(mu);
            auto nu = random1DPointCloud(8,0.1);
            auto nupc = PointCloud::Add(nu);

            show << inNextFrame << pct << mupc << nupc;

            auto mup =mu;auto nup = nu;
            std::sort(mup.begin(),mup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});
            std::sort(nup.begin(),nup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});

            PrimitiveGroup Assignments;
            for (int i = 0;i<N;i++){
                Assignments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta)),100,false,0.005);
            }
            show << inNextFrame << Assignments;
        }
        //show << inNextFrame << PlaceBelow(Image::Add("OT_1D.png",0.7),0.05);
        show << inNextFrame << PlaceBottom(Latex::Add("$\\mathcal{O}(n\\log(n))$!",Options::UPS_default_height_ratio*1.7));
    }

    {
        show << newFrame << Title("Sliced Optimal Transport")->at(UPS::TOP);

        show << inNextFrame << Formula::Add("\\W(\\mu,\\nu)")->at("W2");
        show << inNextFrame << PlaceNextTo(Formula::Add(R"(\approx SW(\mu,\nu) = \int_{\Sp} \W(\proj_\# \mu ,\proj_\# \nu) d\theta)"),1);

        auto mu = randomPointCloud(N);
        auto mupc = PointCloud::Add(mu);
        auto nu = randomPointCloud(N);
        auto nupc = PointCloud::Add(nu);

        show << inNextFrame << mupc << nupc;
        show << CameraView::Add("planar");

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

    {

        auto uniform = Image::Add("uniform.png");
        auto blue_noise = Image::Add("blue_noise.png");

        show << newFrame << Title("Sampling problem")->at(UPS::TOP);
        show << beginCenter << Image::Add("uniform.png") << PlaceNextTo(Image::Add("blue_noise.png"),1) << endCenter;
        show << inNextFrame << PlaceRelative(Formula::Add("\\W(\\mu_1,\\U) > \\W(\\mu_2,\\U)"),CENTER_X,placeY::REL_BOTTOM,0,0.1);
    }

    auto grad = Formula::Add("\\nabla_{x_i} SW^{\\theta} = T^{\\theta}(x_i) - x_i");
    auto descent = Formula::Add("x_i^{n+1} = x_i^n - \\tau \\nabla_{x_i^n} SW^\\theta");

    {
        show << newFrame << Title("Sliced Optimal Transport Sampling")->at(TOP);
        show << inNextFrame << PlaceBelow(Latex::Add("Stochastic Gradient Descent on $\\mu \\mapsto SW(\\mu,\\nu)$"),0.04);
        auto back = show.getCurrentSlide();
        show << CameraView::Add("planar_close");

        {
            auto swgrad = Formula::Add(R"(SW^\theta(\delta_x,\delta_y) = \frac{1}{2}(x-y)^2 \implies \nabla_{x} SW^\theta = ?)");
            vec theta = vec(1,0,0);
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.005);

            vec x = vec(-0.2,0,0);
            vec y = -x;
            auto mu = vecs({x});//random1DPointCloud(8,-0.1);
            auto mupc = PointCloud::Add(mu);
            auto nu = vecs({y});//random1DPointCloud(8,0.1);
            auto nupc = PointCloud::Add(nu);

            show << inNextFrame << PlaceBelow(swgrad) <<  pct << mupc << nupc;
            show << Formula::Add("x")->track([x](){return x;},vec2(0.02,0.02)) << Formula::Add("y")->track([y](){return y;},vec2(0.02,0.02));

            auto mup =mu;auto nup = nu;
            std::sort(mup.begin(),mup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});
            std::sort(nup.begin(),nup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});

            PrimitiveGroup Assignments;
            for (int i = 0;i<1;i++){
                Assignments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta)),100,false,0.005);
            }
            show << Assignments;
            show << inNextFrame << Replace(grad,swgrad);
        }

        show << back << CameraView::Add("planar");
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

        show << CameraView::Add("close_grad",true);

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
        show << inNextFrame << descent->at("advect") << inNextFrame << PointCloud::Add(mu) >> mupc;
    }
    auto grid = Mesh::Add(Options::DataPath+"meshes/tri_grid_50.obj");



    {
        auto offset = 1.5;
        auto T = Title("Non-Euclidean Sliced \\\\ Optimal Transport Sampling");
        show << newFrame << T;
        /*
        show << T->at(UPS::TOP);
        show << inNextFrame << Mesh::Add(Options::DataPath + "meshes/ico_sphere_5.obj")->translate(vec(-offset,0,0));
        show << CameraView::Add("models");
        show << grid->apply([offset](const vec& x) {
            auto u = x(0)*1.5,v = 2*M_PI*x(2);
            return vec(sinh(u)*cos(v) + offset,sinh(u)*sin(v),cosh(u)-1.);
        });
        show << Formula::Add("\\mathbb{S}^2")->at("S2");
        show << Formula::Add("\\mathbb{H}^2")->at("H2");
        */
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
    }
    {
        auto T = Title("Optimization on Manifolds");
        show << newFrame << T << inNextFrame << T->at(TOP);
        show << CameraView::Add("mani_opti");
        auto S = Mesh::Add(Options::DataPath + "meshes/ico_sphere_5.obj",vec(1,1,1),true);
        show << S;

        show << Formula::Add("f : \\mathbb{S}^d \\rightarrow \\mathbb{R}")->at("f_def");
        auto fs = S->eval([](vec x){return x(2);});
        auto func = AddPolyscopeQuantity(S->pc->addVertexScalarQuantity("f",fs));
        show << func;
        show << inNextFrame << PlaceBelow(Formula::Add("\\nabla f(x) \\in TM_x"));

        vec mu = S->getVertices()[2447];
        auto mupc = Point::Add(mu,0.035);
        auto gradf = vec(0,0,0.3);
        gradf -= mu.dot(gradf)*mu;

        vec t1 = vec::Random(),t2 = vec::Random();
        GramSchmidt(mu,t1,t2);
        auto Tp = grid->apply([mu,t1,t2](const vec& x){return vec(mu + t1*x(0)*0.4 + t2*x(2)*0.4);});
        show << Tp->at(0.5);
        auto gradfpc = mupc->addVector(gradf);
        show << mupc << gradfpc << Formula::Add("x")->track([mupc](){return mupc->getCurrentPos();},vec2(-0.02,0.02));
        show << inNextFrame;

        show << descent->at("grad_optim");
        auto question = Latex::Add("?");
        show << inNextFrame << PlaceAbove(question) << inNextFrame;

        auto RGD =  Latex::Add("Riemannian Gradient Descent:\\\\ $x_{n+1} = \\text{Exp}_{x_n}(-\\tau \\nabla f (x_n))$");
        show << RGD->at("RGD");

        auto Exp = Point::Add([mu](TimeObject t){
            auto x = t.inner_time*0.1;
            if (x > 1)
                return vec(0,0,-1);
            return slerp(x,mu,vec(0,0,-1));
        },0.035);

        show << inNextFrame << Exp << inNextFrame >> Exp;

        vec nu = S->getVertices()[1489];
        auto nupc = Point::Add(nu,0.035);

        show << Replace(Formula::Add((R"( x^{n+1}_i = \text{Exp}_{x^n_i}(-\tau\nabla_{x^n_i} SW))")),descent);
        show >> RGD >> question >> gradfpc;
        show << inNextFrame;
        show << PlaceBelow(grad) << inNextFrame;
        show << PlaceBelow(question);
        show << inNextFrame;
        show << nupc << Formula::Add("y")->track([nupc](){return nupc->getCurrentPos();},vec2(0.02,0.02));
        show << Formula::Add("\\text{Exp}^{-1} ?")->at("Log");
        show << inNextFrame;
        auto Log = Formula::Add("\\text{Log}_x(y)");
        show << Replace(Log);
        vec logy = orthoproj(nu-mu,mu)*0.5;
        show << mupc->addVector(logy);
        auto ExpLog = Point::Add([mu,nu](TimeObject t){
            auto x = t.inner_time*0.3;
            if (x > 1)
                return vec(0,0,-1);
            return slerp(x,mu,nu);
        },0.035);

        show << inNextFrame << ExpLog;
        show << inNextFrame >> Log << Replace(Formula::Add((R"( \nabla_{x_i} SW^\theta = \text{Log}_{x_i}(T^\theta (x_i)))")),grad) >> question;
    }

    {
        show << newFrame << Title("NESOTS algorithm")->at(TOP);
        show << Image::Add("nesots_algo.png",0.8)->at("nesots");
        show << CameraView::Add("nesots");
        auto slice= Curve3D::Add([] (scalar t) {return vec(cos(t*2*M_PI),sin(t*2*M_PI),0);},100,true,0.01);
        auto S = Mesh::Add(Options::DataPath + "meshes/ico_sphere_5.obj",vec(1,1,1),true);
        //S->pc->setEdgeWidth(0);
        //2530 1914
        show << inNextFrame;
        show << S;

        vec mu = S->getVertices()[2530];
        vec nu = S->getVertices()[1914];
        auto mupc = Point::Add(mu,0.035);
        auto nupc = Point::Add(nu,0.035);


        show << Formula::Add("\\mu = \\delta_x, \\nu = \\delta_y")->at("measures");
        show <<  mupc << nupc << Formula::Add("x")->track([mupc](){return mupc->getCurrentPos();},vec2(0.02,0.02));

        show << Formula::Add("y")->track([nupc](){return nupc->getCurrentPos();},vec2(0.02,0.02));
        auto slice_label =  Formula::Add("\\theta");
        show << inNextFrame << slice << slice_label->at("slice");

        vec pi_mu = vec(mu(0),mu(1),0).normalized();
        vec pi_nu = vec(nu(0),nu(1),0).normalized();
        auto th = std::acos(pi_mu.dot(pi_nu));
        auto pimupc = Point::Add([mu,pi_mu,pi_nu](const TimeObject& t){
            auto x = t.from_action*0.7;
            if (t.relative_frame_number == 0){
                if (x > 1)
                    return pi_mu;
                return slerp(x,mu,pi_mu);
            }
            if (t.relative_frame_number == 1)
                return pi_mu;
            if (x > 1 || t.relative_frame_number >= 3)
                return pi_nu;
            return slerp(x,pi_mu,pi_nu);
        },0.035);
        auto pinupc = Point::Add([nu,pi_nu](const TimeObject& t){
            auto x = t.inner_time*0.7;
            if (x > 1)
                return pi_nu;
            return slerp(x,nu,pi_nu);
        },0.035);
        vec R = Eigen::AngleAxis(th,vec(0,0,1))*mu;
        show << inNextFrame << pimupc << pinupc;
        auto projlabel = Formula::Add("P^\\theta(x)");
        show << projlabel->track([pimupc](){return pimupc->getCurrentPos();},vec2(0.02,0.02));
        auto T = Curve3D::Add([pi_mu,pi_nu](scalar t){return slerp(t,pi_mu,pi_nu);},30,false,0.013);
        auto projlabel_nu = Formula::Add("P^\\theta(y)");
        show << projlabel_nu->track([pinupc](){return pinupc->getCurrentPos();},vec2(0.02,-0.04));
        show << inNextFrame << T;
        auto Tlabel = Formula::Add("\\text{here, } T(P^\\theta(x)) = P^\\theta(y)");
        show << Tlabel->at("plan");
        auto group_label = Latex::Add(tex::center("$R \\in SO(3)$ \\\\ s.t. $RP^\\theta(x) = P^\\theta(y)$"));



        auto rot_mu = Point::Add([mu,R](TimeObject t){
            auto x = t.inner_time*0.7;
            if (x > 1)
                return R;
            return slerp(x,mu,R);
        },0.035);
        show << inNextFrame << rot_mu;
        auto rotlabel = Formula::Add("Rx");
        show << rotlabel->track([rot_mu](){return rot_mu->getCurrentPos();},vec2(0.02,0.02));
        show << group_label->at("group_action");
        auto grad = mupc->addVector([mu,R](scalar){return Log(mu,R);});
        show << inNextFrame << grad;
        show << inNextFrame >> pimupc >> pinupc >> slice >> T >> rot_mu >> projlabel>> projlabel_nu >> rotlabel >> Tlabel >> group_label >> slice_label;
        show << Formula::Add("\\nabla_x \\text{SW}^\\theta(\\mu,\\nu) = \\text{Log}_x(R(x))")->at("grad");
        //show << Image::Add("logexp.png",1)->at("logexp");
    }
    {
        show << newFrame << Title("Geometric Median for robust SGD")->at(TOP);
        show << Latex::Add("Classic SGD:")->at("SGD");
        show << PlaceBelow(Formula::Add("\\nabla_{x_i} \\mathcal{SW}\\left(\\sum_i \\delta_{x_i},\\nu\\right) \\leftarrow \\frac{1}{L}\\sum_{l = 1}^{L} \\left( T(x_i) - x_i\\right) "));
        auto meangd = Image::Add("mean_GD.png");
        show << meangd->at("meangd");
        show << inNextFrame;
        show << Latex::Add("GeoMed Descent:")->at("GMD");
        show << PlaceBelow(Formula::Add("\\text{GeoMed}(\\{ T(x_i) - x_i\\}_i)"));
        show << inNextFrame;
        show << Image::Add("geomed_GD.png")->at("geomed2");
    }
    {
        show << newFrame << Title("Intrinsic Mesh Sampling")->at(TOP);
        show << Image::Add("mesh_sampling.png");
        show << inNextFrame << Replace(Image::Add("mesh_sampling_cmp.png"));
    }
    {
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
    show << PlaceBelow(Latex::Add("Code Available : baptiste-genest (Github)"));

}

int main(int argc,char** argv)
{
    show.init("NESOTS");

    init();

    polyscope::state::userCallback = [](){
        show.play();
        //ImGui::ShowDemoWindow();
    };
    polyscope::show();
    return 0;
}
