#include "../implicit_uvs_slides.h" 

using namespace slope;

namespace moving {

scalar sdTorus(const vec& p,scalar r1,scalar r2) {
    return (vec2(vec2(p(0),p(1)).norm() - r1,p(2)) - vec2(0,0)).norm() - r2;
}

scalar impl(const vec& x){
    return sdTorus(x,1.,0.25);
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
}

void CreateMovingSlides(slope::Slideshow& show) {
    using namespace moving;
    auto Context = ImplicitUVsSlides::getContext();

    Latex::NewCommand("TM","\\text{T}_{#1}\\text{M}",1);
    Latex::NewCommand("Exp","\\text{Exp}");
    Latex::NewCommand("Log","\\text{Log}");

    auto title = Title("Moving on implicit surfaces");
    show << newFrame <<  title;
    show << newFrame << title->at(TOP);

    auto surface = Point::Add(vec(0.,0.,0.),1.);

    show << CameraView::Add("moving");
    show << surface;
    auto idea = Latex::Add("Idea: Walk on \\emph{approximations} of the surface");
    show << PlaceRelative(idea,title,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);
    show << Formula::Add("\\text{M} = \\{f(x) = 0\\}")->at("impl_formula_2");

    vec x = vec(0.5,0.,0);
    x(2) = height(x);


    auto P = Point::Add(x,0.03);
    show << inNextFrame << P << Latex::Add("p")->at(x,vec2(0.02,-0.02));
    vec n = normal(x)*0.3;
    auto np = P->addVector(n);

    auto E = slope::CompleteBasis(n);

    vec v = -(E.first - E.second).normalized();
    auto vp = P->addVector(vec(v*0.3));

    auto taylor = [E, n, x](const Vertex &V, const TimeObject &t)
    {
        vec v = V.pos;
        if (t.relative_frame_number <= 1)
        {
            vec p = x + (E.first * v(0) + E.second * v(1)) * 0.5 + n * 0.02;
            return p;
        }
        else if (t.relative_frame_number == 2)
        {
            vec p = order2(x, 0.5 * (E.first * v(0) + E.second * v(1)), smoothstep(t.from_action)) + n * 0.02;
            return p;
        }
        vec p = order2(x, 0.5 * (E.first * v(0) + E.second * v(1)), 1.) + n * 0.02;
        return p;
    };

    show << inNextFrame << np << Formula::Add("n(p) = \\nabla f(p)")->at(x+n,vec2(0.02,-0.02));

    auto approx = Context.disk->applyDynamic(taylor);
    show << inNextFrame << approx->at(1.);
    auto O1 = Latex::Add("Order 1 approximation:");
    auto order1_approx = Formula::Add("p + \\tau v");
    show << inNextFrame << vp << Latex::Add("v")->at(x + v*0.3,vec2(-0.02,-0.02));
    show << O1->at("O1") << PlaceRelative(order1_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);

    auto O2 = Latex::Add("Order 2 approximation:");
    auto order2_approx = Formula::Add("p + \\tau v - \\frac{\\tau^2}{2||\\nabla f||} \\langle v, H_f v\\rangle n");
    show << inNextFrame << PlaceRelative(O2,ABS_LEFT,REL_BOTTOM,0.04,0.05) << PlaceRelative(order2_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);

    auto gamma = [v,n,x](scalar t,const TimeObject &time) {
        return vec(order2(x,v*t*0.7*smoothstep(time.inner_time),1) + n*0.04);
    };
    auto W = slope::Curve3D::Add(gamma);
    show << inNextFrame << W;
    show << inNextFrame << PlaceRelative(Formula::Add("\\langle v, H_f v\\rangle \\propto f(x-v) - 2f(x) + f(x+v)"),order2_approx,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.02);
}
