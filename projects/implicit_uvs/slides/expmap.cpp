#include "../implicit_uvs_slides.h" 

using namespace slope;

vec Exp(const vec& p,const vec& v) {
    scalar t = v.norm();
    if (t < 1e-6) return p;
    return p*cos(t) + sin(t)*v/t;
}

vec lemniscate(scalar t) {
    scalar x = cos(t)/(1+sin(t)*sin(t));
    scalar y = sin(t)*cos(t)/(1+ sin(t)*sin(t));
    scalar z = 0.;
    return vec(x,y,z)*0.7;
}

void CreateExpMapSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();
    show << newFrame << Title("Geodesic polar coordinates")->at(TOP);

    vec C = vec(1.5,0,0);
    auto sphere = Point::Add(C,1);
    show << inNextFrame << sphere;

    auto TM = Context.grid->translate(vec(-1.5,0,1.));
    TM->pc->setEdgeWidth(1);
    show << TM;
    show << CameraView::Add("polar_coords");

    show << Latex::Add("$\\TM{p}$")->at("TM0");
    show << Latex::Add("M")->at("M0");
    show << Formula::Add("(r\\cos(\\theta),r\\sin(\\theta))")->at("polar_coordinates");

    vec p = vec(1.5,0,1);
    show << Point::Add(p,0.05);
    show << Formula::Add("p")->at(p,vec2(0.02,-0.02));


    auto img = Context.grid->apply([](const vec& v){
        return vec(Exp(vec(0,0,1),v) + vec(1.5,0,0.04));  
    });
    show << img;
    img->pc->setEdgeWidth(1);

    auto expm = Formula::Add("\\xrightarrow{\\text{Exp}_p}");
    show << expm->at("exp_map");
    auto curve_param = [](scalar t){
        return lemniscate(t);
    };

    auto orig_TpM = Point::Add(vec(-1.5,0,1),0);
    auto v_param = orig_TpM->addVector(curve_param);

    auto curve_M = Point::Add([](scalar t){
        auto p = lemniscate(t);
        return vec(Exp(vec(0,0,1),p) + vec(1.5,0,0.01));
    });

    show << orig_TpM << v_param << curve_M;
    show << inNextFrame << Latex::Add("It is an \\emph{explicit} mapping!")->at("explicit_expm");
    show << inNextFrame >> orig_TpM >> curve_M << PlaceBelow(Formula::Add("\\xleftarrow{\\text{Log}_p}"),expm,0.05);

    auto Exp = Formula::Add("\\text{Exp}_p(v)");
    auto Log = Formula::Add("\\text{Log}_p(x)");
    show << newFrameSameTitle << Exp->at(vec2(0.3,0.3));
    show << Log->at(vec2(0.7,0.3));

    show << inNextFrame << PlaceBelow(Latex::Add(tex::center("From $p$ follow the \\\\ direction $v$")),Exp,0.1);
    show << inNextFrame << PlaceBelow(Latex::Add("Easy!"),0.1);

    show << inNextFrame << PlaceBelow(Latex::Add(tex::center("Shortest path on M \\\\ from $p$ to $x$")),Log,0.1);
    show << inNextFrame << PlaceBelow(Latex::Add("Hard!"),0.1);

    show << inNextFrame << PlaceBottom(Latex::Add("Djikstra?"),0.5,0.1);

}
