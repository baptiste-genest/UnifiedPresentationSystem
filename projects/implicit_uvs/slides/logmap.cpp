#include "../implicit_uvs_slides.h" 

using namespace slope;

scalar r = 0.9;
scalar H = 0.5;

scalar height(const vec& v) {
    return H*(v(0)*v(0)-v(1)*v(1));
}

scalar impl(const vec& x){
    return x(2) - height(x);
}

vec embed(scalar x,scalar y) {
    return vec(x,y,height(vec(x,y,0)));
}

scalar dx = 0.0001;
vec delta(int i) {
    vec d = vec::Zero();
    d(i) = dx;
    return d;
}

vec gradient(const vec& x) {
    vec g;
    for (int i = 0;i<3;i++)
        g(i) = impl(x+delta(i)) - impl(x-delta(i));
    return g/(2*dx);
}

vec normal(const vec& x) {
    return gradient(x).normalized();
}

vec proj_impl(const vec& x) {
    vec g = gradient(x);
    return vec( x - g*impl(x)/g.squaredNorm());
}

vec order2(const vec& x,const vec& v) {
    vec h = v.normalized();
    scalar curv = (impl(x + h*dx) - 2*impl(x) + impl(x - h*dx))/(dx*dx);
    vec g = gradient(x);
    scalar t = v.norm();
    return proj_impl(x + v - t*t*0.5*curv*g.normalized()/g.norm());
}

scalar length(const vecs& X) {
    scalar l = 0;
    for (int i = 0;i<X.size()-1;i++)
        l += (X[i+1]-X[i]).norm();
    return l;
}

vec2 approx_log(const vec& x,const vec& y,const vec& e1,const vec& e2) {
    int N = 10;
    vecs G(N);
    for (int i = 0;i<N;i++) {
        scalar t = scalar(i)/(N-1);
        G[i] = proj_impl(x + (y-x)*t);
    }
    vec n = normal(y);
    vec d = slope::OrthoprojAagainstB(x-y,n).normalized();
    return vec2(d.dot(e1),d.dot(e2))*length(G);
}

void CreateLogMapSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();
    show << newFrame;

    auto title = Title("Ambient heuristic to compute \\\\ Log maps on implicit surfaces");
    show << title;
    show << inNextFrame << TOP;

    auto surface = Context.disk->apply([&](vec v){
        v *= r;
        return vec(v(0),v(1),H*(v(0)*v(0)-v(1)*v(1)));  
    });


    vec x = embed(-0.6,0);
    vec y = embed(0.6,0);

    auto px = Point::Add(x,0.03);
    auto py = Point::Add(y,0.03);

    show << px << py << Formula::Add("x")->at(x,vec2(0.02,-0.02)) << Formula::Add("y")->at(y,vec2(-0.02,-0.02));
    show << surface;
    show << CameraView::Add("logmap_heuristic");

    vec d= (y-x).normalized();

    PrimitiveGroup labels;
    labels << px->addVector(vec(d*0.3)) << Formula::Add("y-x")->at(x+d*0.3,vec2(0.02,-0.02));

    show << inNextFrame << labels;
    vec n = normal(x);

    vec pd = slope::OrthoprojAagainstB(d,n) ;
    labels << px->addVector(vec(pd*0.3 + vec(0,0,0.03)));
    show << inNextFrame << labels;// << Formula::Add("\\Pi_{\\TM{x}}(y-x)")->at(x+d*0.3,vec2(0.02,0.02));

    scalar tmax = 0.5;

    auto gamma = [x,pd,n,tmax](scalar t,const TimeObject &time) {
        if (time.relative_frame_number == 0)
            return vec(order2(x,pd*t*tmax*smoothstep(time.from_action)) + n*0.01);
        return vec(order2(x,pd*t*tmax) + n*0.01);
    };
    auto W = slope::Curve3D::Add(gamma);
    labels << W;
    show << inNextFrame << labels;
    vec x1 = order2(x,pd*tmax);
    auto px1 = Point::Add(x1,0.03);
    labels << px1 << Formula::Add("x_1")->at(x1,vec2(0.,-0.04));
    show << inNextFrame << labels;

    vec nd = (y-x1).normalized();

    labels << px1->addVector(vec(nd*0.3)) << Formula::Add("y-x_1")->at(x1+nd*0.3,vec2(0.02,0.05));

    show << inNextFrame << labels;

    auto geodesic = [x,y](scalar t,const TimeObject& time) {
        vec p = proj_impl(x + (y-x)*t*smoothstep(time.inner_time));
        return p;
    };

    show << inNextFrame >> labels;
    vec mid = proj_impl(x + 0.5*(y-x));
    mid += normal(mid)*0.04;
    show << Curve3D::Add(geodesic);
    show << Formula::Add("\\gamma")->at(mid,vec2(0,-0.02));

    show << Latex::Add("How to compute (polar) coordinates from $\\gamma$?")->at("gamma_cords");

    show << inNextFrame << Latex::Add("$\\text{Length}(\\gamma)$ gives an approximation \\\\ of the geodesic distance $d_M(x,y)$")->at("geo_dist");
    show << CameraView::Add("referential_y",true);

    vec ny = normal(y);
    vec e1 = Eigen::AngleAxisd(-0.5,ny)*OrthoprojAagainstB(x-y,ny).normalized();
    vec e2 = ny.cross(e1);
    e1 += ny*0.04;
    e2 += ny*0.04;

    show << inNextFrame << py->addVector(vec(e1*0.3)) << py->addVector(vec(e2*0.3));
    show << Latex::Add("We compute the angle with \\\\ a reference frame $\\{e_1,e_2\\}$")->at("ref_frame");
    show << Formula::Add("e_1")->at(y+e1*0.3,vec2(-0.02,0.02)) << Formula::Add("e_2")->at(y+e2*0.3,vec2(-0.02,-0.02));

    vec lx = proj_impl(y + 0.1*(x-y));
    scalar cos_theta = (lx-y).normalized().dot(e1);
    scalar sin_theta = (lx-y).normalized().dot(e2);
    scalar th = std::atan2(sin_theta,cos_theta);

    auto arc = [y,e1,e2,th](scalar t) {
        scalar r = 0.25;
        vec p = proj_impl(y + (e1*cos(th*t) + e2*sin(th*t))*r);
        return p;
    };


    show << inNextFrame << Curve3D::Add(arc);


    show << Formula::Add("\\theta")->at(vec(lx),vec2(0.045,0.01));
    show << inNextFrame << Formula::Add("\\text{Log}_y(x) \\approx \\text{Length}(\\gamma) (\\cos(\\theta),\\sin(\\theta)) ")->at("approx_log");


    std::vector<vec2> uvs(surface->getVertices().size());
    for (size_t i = 0;i<uvs.size();i++) {
        vec p = surface->getVertices()[i];
        uvs[i] = approx_log(p,y,e1,e2);
        //std::cout << "UV " << i << " : " << uvs[i].transpose() << std::endl;
    }


    auto pc = surface->pc->addVertexParameterizationQuantity("uv",uvs);
    pc->setCheckerSize(0.05);
    show << inNextFrame << AddPolyscopeQuantity(pc);
    show << inNextFrame << Latex::Add("We call $\\left(y,e_1,e_2\\right)$ \\textbf{the seed \\\\ of the uv-field}")->at("seed");
}