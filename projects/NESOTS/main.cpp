#include "../../src/slope.h"

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

curve_param roundArrowParam(const vec& a ,const vec& b,int side = 1) {
    vec mid = (a+b)/2;
    vec dx = b-mid;
    //int side = rand()%2;
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

vecs opposite(vecs X){
    for (auto& x : X)
        x *= -1.;
    return X;
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

std::shared_ptr<slope::CurveNetwork> plotLines(const vecs& p,scalar l = 2) {
    std::vector<vec> L(p.size()*2);
    slope::Curve3D::edges E(p.size());
    for (int i = 0;i<p.size();i++){
        L[2*i] = p[i]*2;
        L[2*i+1] = -p[i]*2;
        E[i] = {2*i,2*i+1};
    }
    return slope::CurveNetwork::Add(L,E);
}

std::vector<scalar> readLayoutBelong(std::string path,int N) {
    std::vector<scalar> V(N);
    std::ifstream file(path);
    for (int i = 0;i<N;i++)
        file >> V[i];
    return V;
}

vecs sample2DGaussian(int N) {
    //sample 2D gaussian points between -3 and 3

    //define 1D gaussian
    std::normal_distribution<double> dist(0,1);
    std::default_random_engine gen;

    vecs points(N,vec::Zero());
    for (int i = 0;i<N;++i) {
        points[i](0) = dist(gen);
        points[i](2) = dist(gen);
    }
    return points;
}

using VFPC = std::shared_ptr<PolyscopeQuantity<polyscope::PointCloudVectorQuantity>>;
bool runall = false;
void init() {


    Latex::NewCommand("W","\\mathcal{W}");
    Latex::NewCommand("U","\\mathcal{U}");
    Latex::NewCommand("Sp","\\mathbb{S}");
    Latex::NewCommand("Hy","\\mathbb{H}");
    Latex::NewCommand("Pp","\\mathbb{P}");
    Latex::NewCommand("Rd","\\mathbb{R}^d");
    Latex::NewCommand("proj","P^\\theta");
    Latex::UsePackage("cmbright");

    int N = 20;

    {
//        show << Title("Non Euclidean Sliced \\\\ Optimal Transport Sampling")->at(slope::CENTER);
//        show << PlaceBelow(Latex::Add(tex::center("Baptiste Genest \\\\  Nicolas Courty \\\\ David Coeurjolly"),Options::Slope_default_height_ratio*0.8),0.1);
//        show << PlaceBelow(Latex::Add("Eurographics 2024",Options::Slope_default_height_ratio*0.7),0.1);
//        show << PlaceBottomLeft(Latex::Add("aaaaa",Options::Slope_default_height_ratio*0.7),0.01);
        show << Image::Add("welcome_2.jpeg",1);
        show << BackgroudColor::Add(0,0,0);
    }

    {
        show << newFrame << Title("Context")->at(TOP);
        show << PlaceRelative(Latex::Add("We contribute to the \\textcolor{blue}{blue-noise} sampling techniques that use \\textcolor{blue}{optimal transport} by extending those techniques to \\textcolor{blue}{non-Euclidean} domains ($\\Sp^d,\\Hy^d,\\Pp^d$)"),ABS_LEFT,slope::REL_BOTTOM,0.03,0.1);
        show << PlaceBottom(Image::Add("teaser.png"),0.5,0.1);
    }

    {
        show << newFrame << Title("Optimal Transport")->at(slope::TOP);
        show << PlaceBelow(Latex::Add("Discrete to discrete"));
        show << PlaceBottom(Latex::Add(tex::center("Which assignation $\\gamma$ minimizes the effort to move each $x_i$ to $\\gamma(x_i)$?")),0.5,0.1);

        show << Image::Add("OT_1.png");
        show << inNextFrame << Replace(Image::Add("OT_2.png"));

    }
    {
        show << newFrame;
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Kantorovitch 1939], [Peyré et al. 2019], [Villani 2008]"),1,0.01);
        show << Title("Optimal Transport")->at(slope::TOP);
        show << PlaceBelow(Latex::Add("The Wasserstein distance"));
        show << PlaceRelative(Latex::Add("Given two probability measures $\\mu$ and $\\nu$"),placeX::ABS_LEFT,slope::REL_BOTTOM,0.05,0.1);
        show << PlaceRelative(Formula::Add(R"(\W_p^p(\mu,\nu) = \inf_{\gamma \in \Pi(\mu,\nu)} \int d(x,y)^p d\gamma)"),placeX::CENTER_X,slope::REL_BOTTOM,0.05,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("Very importantly, $\\mu$ and $\\nu$ can be discrete or continuous!"),placeX::ABS_LEFT,slope::REL_BOTTOM,0.05,0.1);
        show << PlaceRelative(Image::Add("OT_discrete_conti_2.png"),placeX::CENTER_X,slope::REL_BOTTOM,0.05,0.01);
        show << PlaceRelative(Latex::Add("Image by Gabriel Peyré",Options::Slope_default_height_ratio*0.6),placeX::ABS_RIGHT,slope::REL_BOTTOM,0.01,0.005);
    }
    {
        show << newFrame;
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Kantorovitch 1939]"),1,0.01);
        show << Title("How hard is it?")->at(slope::TOP);
        show << inNextFrame << PlaceRelative(Latex::Add("Exact discrete to discrete: Linear Programming $\\implies \\mathcal{O}(N^3)$"),placeX::ABS_LEFT,REL_BOTTOM,0.051,0.1);
        show << inNextFrame << PlaceRelative(Latex::Add("Trivial in 1 case: 1D-OT"),placeX::ABS_LEFT,REL_BOTTOM,0.05,0.1);
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
                Assignments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta),i%2),100,false,0.005);
            }
            show << inNextFrame << Assignments;
        }
        //show << inNextFrame << PlaceBelow(Image::Add("OT_1D.png",0.7),0.05);
        show << inNextFrame << PlaceBottom(Latex::Add("$\\mathcal{O}(N\\log(N))$",Options::Slope_default_height_ratio*1.5));
    }

    {
        show << newFrame << Title("Sliced Optimal Transport")->at(slope::TOP);

        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Pitié at al. 2005], [Rabin et al. 2011], [Bonnotte 2011], [Bonneel et al. 2015], [Nadjahi et al. 2020]"),1,0.01);
      
        auto mu = randomPointCloud(N);
        auto mupc = PointCloud::Add(mu);
        auto nu = randomPointCloud(N);
        auto nupc = PointCloud::Add(nu);

        show << inNextFrame << mupc << nupc;
        show << CameraView::Add("planar");

        show << inNextFrame;

        int L = 2;
        vecs Theta(L);
        for (auto& x : Theta){
            x = vec::Random();
            x(1) = 0;x.normalize();
        }

        Theta[1] -= Theta[1].dot(Theta[0])*Theta[0];
        Theta[1].normalize();

        for (int i = 0;i<L;i++) {
            vec theta = Theta[i];
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
                Assigments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta),i%2),100,false,0.002);
            }
            show <<inNextFrame << Assigments;
            show << inNextFrame >> pct >> m >> n >> Assigments;
        }
        show << Formula::Add(R"(\frac{1}{L} \sum_i^L\W(P^{\theta_i}_\# \mu ,P^{\theta_i}_\# \nu) )" + tex::AbelowB("L\\rightarrow \\infty","\\longrightarrow"))->at("W2") << inNextFrame << PlaceNextTo(Formula::Add(R"(\int_{\Sp} \W(\proj_\# \mu ,\proj_\# \nu) d\theta = \textcolor{red}{SW(\mu,\nu)})"),1);
        show << inNextFrame << PlaceNextTo(Formula::Add("\\approx \\W(\\mu,\\nu)"),1);
        show << newFrame << Title("Sampling Continuous Densities")->at(TOP);
        show << inNextFrame << PlaceRelative(Latex::Add("We approximate semi-discrete OT by targeting a large number of samples:"),ABS_LEFT,REL_BOTTOM,0.02,0.12);
        auto app = Formula::Add("\\approx");
        show  << app->at(0.5,0.7) << PlaceRelative(Image::Add("gaussian_density.png"),app,slope::REL_LEFT,placeY::SAME_Y) << PlaceRelative(Image::Add("gaussian_samples.png"),app,slope::REL_RIGHT,placeY::SAME_Y);
        show << PlaceNextTo(Formula::Add(R"(\tilde{\nu} = \frac{1}{N}\sum_{i=1}^N \delta_{y_i} )"),1,0.05) << PlaceBelow(Formula::Add(R"(y_i \sim \nu)"),0.03) << PlaceLeft(Formula::Add("\\nu"),0.7,0.18);

    }


    {

        auto uniform = Image::Add("uniform.png");
        auto blue_noise = Image::Add("blue_noise.png");

        show << newFrame << Title("The Blue Noise Sampling Problem")->at(slope::TOP);
        show << inNextFrame << Formula::Add("\\mu_1")->at("mu1") << Formula::Add("\\mu_2")->at("mu2");
        show << Formula::Add("\\U")->at("unif_den");// << Latex::Add(tex::center("$\\mu_1, \\mu_2 \\sim \\U$,\\\\ where $\\U$ \\\\ is the uniform\\\\ distribution"))->at("follow_u");
        show << Formula::Add(R"(\mu = \frac{1}{N}\sum_{i=1}^N \delta_{x_i} )")->at("discrete_mu");
        show << beginCenter << Image::Add("unif.png") << PlaceNextTo(Image::Add("uniform.png"),1) << PlaceNextTo(Image::Add("blue_noise.png"),1) << endCenter;
        show << inNextFrame << PlaceRelative(Formula::Add("\\W(\\mu_1,\\U) > \\W(\\mu_2,\\U)"),CENTER_X,placeY::REL_BOTTOM,0,0.08);
        show << inNextFrame << PlaceBelow(Latex::Add("Idea: Optimize $\\mu$ by gradient descent on $SW(\\mu,\\U)$!",slope::Options::Slope_default_height_ratio*1.4),0.05);
    }

    auto grad = Formula::Add("\\nabla_{x_i} SW^{\\theta} = T^{\\theta}(x_i) - x_i");
    auto descent = Formula::Add("x_i^{n+1} = x_i^n - \\tau \\nabla_{x_i^n} SW");

    {
        show << newFrame;
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Paulin at el. 2020]"),1,0.01);
        show << Title("Sliced Optimal Transport Sampling")->at(TOP);

        show << inNextFrame << PlaceBelow(Latex::Add("Stochastic Gradient Descent on $\\mu \\mapsto SW(\\mu,\\U)$"),0.04);
        auto back = show.getCurrentSlide();
        show << CameraView::Add("planar_close");

        {
            auto swgrad = Formula::Add(R"(SW^\theta(\delta_x,\delta_y) = \frac{1}{2}(x-y)^2 \implies \nabla_{x_i} SW^\theta = x_i - T^{\theta}(x_i))");
            vec theta = vec(1,0,0);
            auto pct = Curve3D::Add(vecs{-theta*3,theta*3},false,0.005);

            vec x = vec(-0.2,0,0);
            vec y = -x;
            auto mu = vecs({x});//random1DPointCloud(8,-0.1);
            auto mupc = PointCloud::Add(mu);
            auto nu = vecs({y});//random1DPointCloud(8,0.1);
            auto nupc = PointCloud::Add(nu);

            show << inNextFrame << PlaceBelow(swgrad) <<  pct << mupc << nupc;
            show << Formula::Add("x")->track([x](){return x;},vec2(0.02,0.02)) << Formula::Add("y=T(x)")->track([y](){return y;},vec2(0.02,0.02));

            auto mup =mu;auto nup = nu;
            std::sort(mup.begin(),mup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});
            std::sort(nup.begin(),nup.end(),[theta](const vec& x,const vec& y) {return x.dot(theta) < y.dot(theta);});

            PrimitiveGroup Assignments;
            for (int i = 0;i<1;i++){
                Assignments << Curve3D::Add(roundArrowParam(proj(mup[i],theta),proj(nup[i],theta),1),100,false,0.005);
            }
            show << Assignments;
        }

        show << back << CameraView::Add("planar");
        auto mu = randomPointCloud(N);

        mu[0] = vec(-0.4,0,0.4);

        auto mupc = PointCloud::Add(mu);
        auto NU = randomPointCloud(N*20);
        auto NUpc = PointCloud::Add(NU);

        show << inNextFrame << mupc << NUpc;

        /*
        auto approx = Latex::Add(tex::center("We approximate \\\\ the target distribution $\\nu$ \\\\ by a large \\\\ number of samples: \\\\ $\\tilde{\\nu} = \\sum_i^N \\delta_{y_i},$ \\\\ where $y_i \\sim \\nu$"));
        show << inNextFrame << approx->at("U");
        */



        show << inNextFrame << NUpc->at(0.5) << inNextFrame;

        int K = 4;

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
                    Assigments << Curve3D::Add(roundArrowParam(proj(mup[i].first,theta),proj(nup[i].first,theta),i%2),100,false,0.002);
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

        show << PlaceLeft(Formula::Add("\\nabla_{x_i} SW" + tex::AbelowB("L\\rightarrow\\infty","\\longleftarrow") + "\\frac{1}{L}\\sum_{\\theta}\\nabla_{x_i} SW^\\theta" ),0.5,0.03);

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

        show << newFrameSameTitle << PlaceBelow(Latex::Add("Results"),0.04);
        show << PlaceBelow(Gif::Add("R2_sampling.gif",20,1.1),0.1);
    }
    auto grid = Mesh::Add(Options::DataPath+"meshes/tri_grid_50.obj");



    {
        auto offset = 1.5;
        auto T = Title("Non-Euclidean Sliced \\\\ Optimal Transport Sampling");
        show << newFrame;
        show << T->at(slope::TOP);
        show << Image::Add("nesots_model2.png",1)->at("models");
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Bonet et al. 2023]"),1,0.01);
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

        auto descent = Formula::Add("\\Rd: x_i^{n+1} = x_i^n - \\tau \\nabla_{x_i^n} SW");
        show << descent->at("grad_optim");
        show << inNextFrame;

        auto RGD =  Latex::Add(tex::center("\\color{gray}\\textit{Riemannian Gradient Descent}:\\\\ $x_{n+1} = \\text{Exp}_{x_n}(-\\tau \\nabla f (x_n))$"));
        show << RGD->at("RGD",0.7);

        auto Exp = Point::Add([mu](TimeObject t){
            auto x = t.inner_time*0.1;
            if (x > 1)
                return vec(0,0,-1);
            return slerp(x,mu,vec(0,0,-1));
        },0.035);

        show << inNextFrame << Exp << inNextFrame >> Exp;

        vec nu = S->getVertices()[1489];
        auto nupc = Point::Add(nu,0.035);

        show << PlaceRelative(Formula::Add((R"(\color{blue} \text{manifolds: } x^{n+1}_i = \text{Exp}_{x^n_i}(-\tau\nabla_{x^n_i} SW))")),descent,ABS_LEFT,REL_BOTTOM);
        show >> gradfpc;
        show << inNextFrame;

        auto grad = Formula::Add("\\Rd: \\nabla_{x} SW^{\\theta} = x - T^{\\theta}(x)");
        show << PlaceBelow(grad,0.1) << inNextFrame;
        show << inNextFrame;
        show << nupc << Formula::Add("y")->track([nupc](){return nupc->getCurrentPos();},vec2(0.02,0.02));
        show << inNextFrame;
        auto Log = Formula::Add("\\text{Log}_x(y)");
        show << Log->at("Log");
        vec logy = orthoproj(nu-mu,mu)*0.5;
        show << mupc->addVector(logy);
        auto ExpLog = Point::Add([mu,nu](TimeObject t){
            auto x = t.inner_time*0.3;
            if (x > 1)
                return vec(0,0,-1);
            return slerp(x,mu,nu);
        },0.035);

        show << inNextFrame << ExpLog;
        show << inNextFrame >> Log << PlaceBelow(Formula::Add((R"(\color{blue}\text{manifolds: } \nabla_{x_i} SW^\theta = \text{Log}_{x_i}(T^\theta (x_i)))")),grad);
    }

    auto S = Mesh::Add(Options::DataPath + "meshes/ico_sphere_5.obj",vec(1,1,1),true);

    {
        show << newFrame << Title("NESOTS Algorithm")->at(TOP);
        show << PlaceRight(Image::Add("nesots_algo.png",0.85),0.55,0.02);
        show << CameraView::Add("nesots");
        auto slice= Curve3D::Add([] (scalar t) {return vec(cos(t*2*M_PI),sin(t*2*M_PI),0);},100,true,0.01);
        //S->pc->setEdgeWidth(0);
        //2530 1914
        show << inNextFrame;
        show << S;
        show << PlaceBottomLeft(Formula::Add("\\Sp^d"),0.05);
        auto cursor = Image::Add("highlight.png",1.1);
        cursor->setDepth(1);

        vec mu = S->getVertices()[2530];
        vec nu = S->getVertices()[1914];
        auto mupc = Point::Add(mu,0.035);
        auto nupc = Point::Add(nu,0.035);


        show << Formula::Add("\\mu = \\delta_x, \\nu = \\delta_y")->at("measures");
        show <<  mupc << nupc << Formula::Add("x")->track([mupc](){return mupc->getCurrentPos();},vec2(0.02,0.02));

        show << Formula::Add("y")->track([nupc](){return nupc->getCurrentPos();},vec2(0.02,0.02));
        auto slice_label =  Formula::Add("\\theta");
        show << inNextFrame << slice << slice_label->at("slice") << cursor->at("cursor0");

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
        show << inNextFrame << pimupc << pinupc << cursor->at("cursor1");
        auto projlabel = Formula::Add("P^\\theta(x)");
        show << projlabel->track([pimupc](){return pimupc->getCurrentPos();},vec2(0.02,0.04));
        auto T = Curve3D::Add([pi_mu,pi_nu](scalar t){return slerp(t,pi_mu,pi_nu);},30,false,0.018);
        auto projlabel_nu = Formula::Add("P^\\theta(y)");
        show << projlabel_nu->track([pinupc](){return pinupc->getCurrentPos();},vec2(0.02,-0.04));
        show << inNextFrame << T << cursor->at("cursor2");
        auto Tlabel = Formula::Add("T(P^\\theta(x)) = P^\\theta(y)");
        show << Tlabel->at("plan");
        auto group_label = Latex::Add(tex::center("Find $\\Gamma_\\theta$, \\\\ s.t. $\\Gamma_{\\theta}(P^\\theta(x)) = P^\\theta(y)$"));



        auto rot_mu = Point::Add([mu,R](TimeObject t){
            auto x = t.inner_time*0.7;
            if (x > 1)
                return R;
            return slerp(x,mu,R);
        },0.035);
        show << inNextFrame << rot_mu << cursor->at("cursor3");
        auto rotlabel = Formula::Add("\\Gamma_\\theta(x)");
        show << rotlabel->track([rot_mu](){return rot_mu->getCurrentPos();},vec2(0.02,0.02));
        show << group_label->at("group_action");
        auto grad = mupc->addVector([mu,R](scalar){return Log(mu,R);});
        show << inNextFrame << grad << cursor->at("cursor4");
        show << inNextFrame;// >> pimupc >> pinupc >> slice >> T >> rot_mu >> projlabel>> projlabel_nu >> rotlabel >> Tlabel >> group_label >> slice_label;
        show << Formula::Add("\\nabla_x \\text{SW}^\\theta(\\mu,\\nu) = \\text{Log}_x(\\Gamma_\\theta((x))")->at("grad");
        show << newFrame;

        show << Title("Results")->at(TOP) << PlaceLeft(Gif::Add("sphere_sampling.gif",20,1.3,false),0.6,0.05);
        show << PlaceRight(Image::Add("compare_sphere.png",1.3),0.6,0.1);
        //show << Image::Add("logexp.png",1)->at("logexp");
    }
    {
        show << newFrame << Title("Generic and closed form formulas")->at(TOP) << Latex::Add(R"(
% Please add the following required packages to your document preamble:
% \usepackage{graphicx}
\begin{table}[]
\resizebox{\textwidth}{!}{%
\begin{tabular}{cccc}
\hline
                    & $\mathbb{R}^d$ & $\mathbb{S}^d$                              & $\mathbb{H}^d$ (Lorentz Model)                                 \\ \hline
$\textbf{Exp}_x(v)$ & $x+v$          & $\cos(||v||)x + \sin(||v||)\frac{v}{||v||}$ & $\text{cosh}(||v||_L)x + \text{cosh}(||v||_L)\frac{v}{||v||} $ \\ \hline
$\textbf{Log}_x(y)$ &
  $y-x$ &
  $\frac{\Pi_{TM_x}(y-x)}{||\Pi_{TM_x}(y-x)||}d(x,y)$ &
  $\frac{\text{arccosh}(-\langle x,y\rangle_L)}{\sqrt{\langle x,y \rangle_L^2-1}}(y+\langle x,y\rangle_L x)$ \\ \hline
$\textbf{P}^\theta(x)$ &
  $\theta \langle \theta , x \rangle$ &
  $\frac{\Pi^\theta(x)}{||\Pi^\theta(x)||}$ &
  $\frac{\Pi^\theta(x)}{\sqrt{-\langle \Pi^\theta(x),\Pi^\theta(x) \rangle_L}}$ \\ \hline
\end{tabular}%
}
\end{table}
)",Options::Slope_default_height_ratio*1);
        show << PlaceBottom(Latex::Add("See details in paper."));
    }
    {
        show << newFrame << Title("Geometric Median for robust SGD")->at(TOP);
        show << Image::Add("sphere_density.png")->at("density_on_sphere");
        show << Latex::Add("Classic SGD:")->at("SGD");
        show << PlaceBelow(Formula::Add("\\nabla_{x_i} SW \\leftarrow \\frac{1}{L}\\sum_{l = 1}^{L} \\nabla_{x_i}SW^{\\theta_l}"));
        auto meangd = Image::Add("mean_GD.png");
        show << meangd->at("meangd");
        show << inNextFrame;
        show << Latex::Add("GeoMed Descent:")->at("GMD");
        show << PlaceBelow(Formula::Add("\\text{GeoMed}(\\{\\nabla_{x_i}SW^{\\theta_l}\\}_l)"),0.05);
        show << inNextFrame;
        show << Image::Add("geomed_GD.png")->at("geomed2");
    }

    {
        show << newFrame << Title("Applications");
        //show << inNextFrame << PlaceBelow(Latex::Add("Our hope : sampling curved domains $\\implies$ easier to apply to Geometry ! "),0.1);
    }

    {
        show << newFrame << Title("Intrinsic Mesh Sampling")->at(TOP);
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Bobenko et al. 2015],[Gillespie et al. 2021],[Schmidt et al. 2020]"),1,0.01);
        //show << PlaceBelow(Formula::Add(R"(\text{Mesh} \rightarrow \Sp\text{ or }\Hy \rightarrow \text{NESOTS} \rightarrow \text{Mesh})"),0.05);
        show << Image::Add("mesh_sampling_full.png");
        //show << PlaceBelow(Formula::Add("g=0"));
        show << newFrameSameTitle << PlaceBelow(Formula::Add("g > 1"));
        show << CameraView::Add("macaca");
        mapping N = [] (vec x) {
            return vec(0.5*x);
        };
        auto M = Mesh::Add(Options::ProjectPath + "macaca.obj")->apply(N);
        show << M << Formula::Add("\\mathbb{R}^3",Options::Slope_default_height_ratio*1.5)->at("R3");
        show << inNextFrame;
        show  << Formula::Add("\\Hy^2",Options::Slope_default_height_ratio*1.5)->at("H2");
        for (int i = 1;i<=3;i++) {
            PrimitiveGroup L;
            L << PointCloud::Add(readPointCloud(Options::ProjectPath + "mu_"+std::to_string(i) + ".pts"))->apply(N);
            L << Mesh::Add(Options::ProjectPath + "layout_"+std::to_string(i) + ".obj");
            L << PointCloud::Add(readPointCloud(Options::ProjectPath + "mu_layout_"+std::to_string(i) + ".pts"));
            auto pc_quantity = AddPolyscopeQuantity<polyscope::SurfaceFaceScalarQuantity>(M->pc->addFaceScalarQuantity(std::to_string(i),readLayoutBelong(Options::ProjectPath + "onlayout_"+std::to_string(i) + ".dat",M->getFaces().size())));
            L << pc_quantity;
            show << L;
            if (i < 3)
                show << inNextFrame >> L;
        }
        show << newFrameSameTitle << Gif::Add("macaca_sampling.gif",14,1.3);
        show << newFrameSameTitle << Image::Add("mesh_sampling_cmp.png");
        scalar l = 0.08;
        show << Latex::Add("ours")->at("ours") << PlaceNextTo(Latex::Add("FP"),1,l) << PlaceNextTo(Latex::Add("PDS"),1,l) << PlaceNextTo(Latex::Add("CVT"),1,l-0.0005) << PlaceNextTo(Latex::Add("\\textit{pcf}"),1,0.14);
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Peyré et al. 2006],[Liu et al. 2009]"),1,0.01);
        show << newFrameSameTitle << Image::Add("non_unif_mesh_sampling.png");
    }
    {
        show << newFrame << Title("Projective Plane Sampling");
        show << CameraView::Add("line_sampling");
        show << inNextFrame << slope::TOP;
        show << PlaceBelow(Latex::Add("How to sample lines?"));
        show << S;
        auto fibo = readPointCloud(slope::Options::ProjectPath+"fibo.pts");
        auto fibopc = PointCloud::Add(fibo,0.05);

        auto hearth_eyes = Image::Add("thumbs_up.png",0.45);
        auto pts = Formula::Add("\\left \\{x_i\\right\\}_i");
        auto puke = Image::Add("thumbs_down.png",0.45);
        auto proj_pts = Formula::Add("\\left \\{x_i\\right\\}_i \\cup \\left \\{-x_i\\right\\}_i");

        auto sample_sphere = Latex::Add("Sampling $\\Sp^2$");
        show << sample_sphere->at("space");

        show << fibopc;
        show << pts->at("fibo") << PlaceNextTo(hearth_eyes,1);
        show << inNextFrame;
        auto fibo_lines = plotLines(fibo);
        show << fibo_lines;
        auto mfibo = PointCloud::Add(opposite(fibo),0.05);
        show << inNextFrame >> fibo_lines << mfibo;
        show << PlaceBelow(puke,hearth_eyes) << PlaceNextTo(proj_pts,0);
        show << inNextFrame;


        auto good = readPointCloud(slope::Options::ProjectPath+"good.pts");
        auto goodpc = PointCloud::Add(good,0.05);
        auto mgood = PointCloud::Add(opposite(good),0.05);
        show >> fibopc >> mfibo << goodpc << mgood >> pts >> proj_pts >> hearth_eyes >> puke;
        show << pts->at("fibo") << PlaceNextTo(puke,1) <<  PlaceBelow(hearth_eyes,puke) << PlaceNextTo(proj_pts,0);
        show << Replace(Latex::Add("Sampling $\\mathbb{P}^2$"),sample_sphere);
        show << inNextFrame << plotLines(good);

        show << newFrameSameTitle;
        show << PlaceBelow(Latex::Add("Rotation sampling"));
        show << inNextFrame << Latex::Add("Idea: sampling unit quaternions $\\subset \\mathbb{S}^3$")->at("unit_quat");
        show << Latex::Add("But since, $q^{-1}\\Vec{x}q = (-q)^{-1}\\Vec{x}(-q)$, we need projective sampling!")->at("quat");
        show << inNextFrame << Gif::Add("rotation_sampling.gif")->at("rot2");
        show << inNextFrame << Image::Add("visibility.png",0.8)->at("visibility");
        show << PlaceBottomLeft(Image::Add("article.png",.3),0.01)     ;
        show << PlaceNextTo(Latex::Add("\\scriptsize \\color{orange}[Alexa 2022]"),1,0.01);

        show << newFrameSameTitle << PlaceBelow(Latex::Add("And much more!"));
        show << Image::Add("line_sampling.png",1.4)->at("line");
        show << Latex::Add("Sampling cartesian equations:")->at("cartesian") << PlaceBelow(Formula::Add("ax+by+c = 0"));
    }

    {
        show << newFrame << Title("Conclusion")->at(TOP);
        show <<
            PlaceRelative(Latex::Add(
                                  tex::enumerate(
                                      "We introduced NESOTS, a general algorithm for\\\\ blue-noise sampling on non-Euclidean geometries",
                                      "It allows us to intrinsincly sample meshes",
                                      "Projective sampling appears to be new and promising")
                                  +"\n Future works: \n " +
                                  tex::enumerate(
                                      "Intrinsic remeshing",
                                      "Extension to other domains (e.g. Lie groups)",
                                                                             "Order 2 methods"
                                                                             )),ABS_LEFT,REL_BOTTOM,0.04,0.15);
        show << Image::Add("sphere_sampling.png",0.5)->at("ccl_teaser1");
        show << Image::Add("duck.png")->at("ccl_teaser2");
        show << Image::Add("rot_sampling.png",0.5)->at("ccl_teaser3");
        //        show << inNextFrame << PlaceBelow(Latex::Add("We showed how to apply it to various geometries"),0.1);
    }

    show << newFrame << Title("Thank you for your attention");
    show << PlaceBelow(Latex::Add("Code Available: \\url{https://github.com/baptiste-genest/NESOTS}"));
    show << Image::Add("ANR.jpg",0.6)->at("ANR") << PlaceNextTo(Image::Add("QR_code.png",0.1),1,0.1);
    show << Image::Add("logo_cnrs.png",0.3)->at("cnrs") << PlaceNextTo(Image::Add("logo_liris.png"),1,0.1)<< PlaceNextTo(Image::Add("logo_irisa.png"),1,0.1);

}

int main(int argc,char** argv)
{
    show.init("NESOTS",argc,argv);

    srand(5);
    init();

    polyscope::state::userCallback = [](){
        show.play();
        //ImGui::ShowDemoWindow();
    };
    polyscope::show();
    return 0;
}
