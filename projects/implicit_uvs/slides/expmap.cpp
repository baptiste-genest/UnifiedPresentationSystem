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
    show << newFrame << Title("A local coordinate system")->at(TOP);

    vec C = vec(1.5,0,0);
    auto sphere = Point::Add(C,1);
    show << inNextFrame << sphere;

    auto TM = Context.grid->translate(vec(-1.5,0,1.));
    TM->pc->setEdgeWidth(0.8);
    show << TM;
    show << CameraView::Add("polar_coords");

    show << Latex::Add("$\\TM{p}$")->at("TM0");
    show << PlaceBelow(Latex::Add("(uv space)"));
    show << Latex::Add("M")->at("M0");
    show << PlaceBelow(Latex::Add("(surface)"));
    show << Formula::Add("(r\\cos(\\theta),r\\sin(\\theta))")->at("polar_coordinates");

    vec p = vec(1.5,0,1);
    show << Point::Add(p,0.05);
    show << Formula::Add("p")->at(p,vec2(0.02,-0.02));


    auto img = Context.grid->apply([](const vec& v){
        return vec(Exp(vec(0,0,1),v) + vec(1.5,0,0.04));  
    });
    show << img;
    img->pc->setEdgeWidth(0.8);
    img->pc->setSurfaceColor(TM->pc->getSurfaceColor());

    auto expm = Formula::Add("\\xrightarrow{\\text{Exp}_p}");
    show << expm->at("exp_map");
    auto curve_param = [](scalar t){
        return lemniscate(t);
    };

    auto orig_TpM = Point::Add(vec(-1.5,0,1),0);
    auto v_param = orig_TpM->addVector(curve_param);
    auto v_label = Point::Add(curve_param,0.);


    auto curve_M = Point::Add([](scalar t){
        auto p = lemniscate(t);
        return vec(Exp(vec(0,0,1),p) + vec(1.5,0,0.01));
    });
    v_param->q->setVectorColor(curve_M->pc->getPointColor());

    show << orig_TpM << v_param << curve_M << v_label;
    auto v_label_text = Formula::Add("v")->track([v_label](){return vec(v_label->getCurrentPos() + vec(-1.5,0,1));},vec2(0.02,0.02));
    auto exp_label = Formula::Add("\\Exp_p(v)")->track([curve_M](){return curve_M->getCurrentPos();},vec2(0.02,0.02));
    show << v_label_text << exp_label;
    show << inNextFrame << Latex::Add("It is an \\emph{explicit} mapping!")->at("explicit_expm");
    show << inNextFrame >> curve_M >> v_param >> v_label_text.first >> exp_label.first;
    vec target = ExpSphere(vec(0,0,1),vec(1,1,0)*0.4) + vec(1.5,0,0);
    vec source = vec(1.3,0,10);

    auto ray = Curve3D::Add([source,target](scalar t) {
        return vec(source + (target-source)*t);
    });

    auto hit = vec(target + vec(0,0,0.01));
    show << inNextFrame << ray << Point::Add(hit,0.05) << Formula::Add("x")->at(hit,vec2(0.02,-0.02));
    show << inNextFrame << PlaceBelow(Formula::Add("\\xleftarrow{\\Log_p}"),expm,0.05);
    show << orig_TpM->addVector(vec(0.4,0.4,0.)) << Formula::Add("\\Log_p(x)")->at(vec(-1.5 + 0.41,0.41,1.),vec2(0.02,-0.02));

    auto Exp = Formula::Add("\\Exp_p(v)");
    auto Log = Formula::Add("\\Log_p(x)");
    show << newFrameSameTitle << Exp->at(vec2(0.3,0.3));
    show << Log->at(vec2(0.7,0.3));

    show << inNextFrame << PlaceBelow(Latex::Add(tex::center("From $p$ follow the \\\\ direction $v$")),Exp,0.1);
    show << PlaceBelow(Latex::Add("Easy!"),0.1);

    show << inNextFrame << PlaceBelow(Latex::Add(tex::center("Shortest path on M \\\\ from $x$ to $p$")),Log,0.1);
    show << PlaceBelow(Latex::Add("Hard!"),0.1);

    show << inNextFrame << PlaceBottom(Latex::Add("Without a mesh, no shortest path algorithm!"),0.5,0.1);

}
