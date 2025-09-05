#include "../implicit_uvs_slides.h" 

using namespace slope;

namespace moving {

scalar sdTorus(const vec& p,scalar r1,scalar r2) {
    return (vec2(vec2(p(0),p(1)).norm() - r1,p(2)) - vec2(0,0)).norm() - r2;
}

scalar impl(const vec& x){
    return sdTorus(x,1.,0.5);
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

vec order2(const vec& x,const vec& v,scalar alpha) {
    vec h = v.normalized();
    scalar curv = (impl(x + h*dx) - 2*impl(x) + impl(x - h*dx))/(dx*dx);
    vec g = gradient(x);
    scalar t = v.norm();
    return x + v - t*t*0.5*curv*g.normalized()/g.norm()*alpha;
}

vec proj_impl(const vec& x) {
    vec g = gradient(x);
    return vec( x - g*impl(x)/g.squaredNorm());
}
}

void CreateMovingSlides(slope::Slideshow& show) {
    using namespace moving;
    auto Context = ImplicitUVsSlides::getContext();

    Latex::NewCommand("TM","\\text{T}_{#1}\\text{M}",1);
    Latex::NewCommand("Exp","\\text{Exp}");
    Latex::NewCommand("Log","\\text{Log}");

    auto title = Title("Moving along implicit surfaces");
    show << newFrame <<  title;
    show << newFrame << title->at(TOP);

    auto surface = Context.torus->copy();

    show << CameraView::Add("moving_far");
    show << surface;
    show << Formula::Add("\\text{M} = \\{f(x) = 0\\}")->at("impl_formula_2");

    vec x = proj_impl(vec(-1.3,0.,0.5));
    //x(2) = height(x);

    auto P = Point::Add(x,0.03);

    vec n = normal(x)*0.3;
    auto np = P->addVector(n);

    auto E = slope::CompleteBasis(n);

    vec v = -(E.first - E.second).normalized();
    auto vp = P->addVector(vec(v*0.3));

    auto walk = Latex::Add("We would like to walk along the surface from a point $p \\in M$ in a tangential direction $v$.");
    show << PlaceRelative(walk,title,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);
    vp->q->setVectorColor(glm::vec3(1,0,0));

    show << P << Latex::Add("p")->at(x,vec2(0.02,-0.02));
    show << vp << Latex::Add("v")->at(x + v*0.3,vec2(-0.02,-0.02));
    //show << inNextFrame;
    auto idea = Latex::Add("No general closed form formula... So: \\\\ Walk on \\emph{approximations} of the surface");
    show << PlaceRelative(idea,walk,slope::ABS_LEFT,slope::REL_BOTTOM,0.02,0.05);

    auto taylor = [E, n, x](const Vertex &V, const TimeObject &t)
    {
        vec v = V.pos;
        if (t.relative_frame_number <= 2)
        {
            vec p = x + (E.first * v(0) + E.second * v(1)) * 0.5 + n * 0.01;
            return p;
        }
        else if (t.relative_frame_number == 3)
        {
            vec p = order2(x, 0.5 * (E.first * v(0) + E.second * v(1)), smoothstep(t.from_action)) + n * 0.01;
            return p;
        }
        vec p = order2(x, 0.5 * (E.first * v(0) + E.second * v(1)), 1.) + n * 0.01;
        return p;
    };

    show << inNextFrame << CameraView::Add("moving_close",true);

    auto approx = Context.disk->applyDynamic(taylor);
    auto TM = Formula::Add("\\TM{p}");
    show << inNextFrame << approx->at(0.9) << TM->at(vec(x - E.second*0.5),vec2(0.02,-0.02));

    auto normal_legend = Formula::Add("n(p) = \\nabla f(p)");
    show << np << normal_legend->at(x+n,vec2(0.02,-0.02));
    
    auto O1 = Latex::Add("Order 1 approximation:");
    auto order1_approx = Formula::Add("p + \\tau v");
    show << inNextFrame << O1->at("O1") << PlaceRelative(order1_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);

    show >> TM;

    auto O2 = Latex::Add("Order 2 approximation:");
    auto order2_approx = Formula::Add("p + \\tau v - \\frac{\\tau^2 }{2||\\nabla f||}\\partial^2_v f n");
    show << inNextFrame << PlaceRelative(O2,ABS_LEFT,REL_BOTTOM,0.04,0.05) << PlaceRelative(order2_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);

    auto gamma = [v,n,x](scalar t,const TimeObject &time) {
        return vec(order2(x,v*t*0.5*smoothstep(time.inner_time),1) + n*0.04);
    };
    show << inNextFrame << CameraView::Add("moving_far",true);
    auto W = slope::Curve3D::Add(gamma);
    W->pc->setColor(glm::vec3(1,0,0));
    show << inNextFrame << W;
    //show << inNextFrame << PlaceRelative(Formula::Add("\\langle v, H_f v\\rangle = \\frac{f(x-hv) - 2f(x) + f(x+hv)}{h^2} + o(h^2)"),order2_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);
}
