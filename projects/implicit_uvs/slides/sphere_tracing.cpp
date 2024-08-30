#include "../implicit_uvs_slides.h" 

using namespace slope;

scalar sdTorus(const vec& p,scalar r1,scalar r2) {
    return (vec2(vec2(p(0),p(1)).norm() - r1,p(2)) - vec2(0,0)).norm() - r2;
}

void CreateSphereTracingSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();
    show << newFrame;

    show << Title("Rendering Implicit surfaces")->at(TOP);
    show << PlaceBelow(Latex::Add("Sphere tracing"),0.01);
    
    auto T = slope::Mesh::Add(Options::DataPath+"meshes/torus.obj",1,true);
    T->pc->setEdgeWidth(0);

    show << T;
    show << CameraView::Add("sphere_tracing");

    vec p = vec(2,0,1.5);
    vec target = vec(-0.5,0,0);
    vec d = (target-p).normalized();

    auto pt = Point::Add(p,0.05);
    auto dir = pt->addVector(vec(d*0.4));

    show << pt << dir;

    vec ref = vec(-1,0.,0.5);
    show << Point::Add(ref) << Formula::Add("y")->at(ref,vec2(0.02,-0.02));

    auto ray = [p,target](scalar t,const TimeObject &time) {
        if (time.relative_frame_number == 0)
            return vec(p + (target-p)*t*smoothstep(time.from_action));
        return vec(p + (target-p)*t);
    };

    auto ray_param = Curve3D::Add(ray);
    show << inNextFrame << ray_param;


    show << Point::Add(target) << Formula::Add("x")->at(target,vec2(0.02,0.02));

    auto geodesic = [] (scalar t,const TimeObject& time) {
        float angle = M_PI_2;
        if (time.relative_frame_number == 0)
            angle *= smoothstep(time.from_action);
        vec p = vec(cos(-t*angle),0,sin(t*angle))*0.5 + vec(-1,0,0);
        return p;
    };
    show << inNextFrame << Curve3D::Add(geodesic);

    show << inNextFrame << Latex::Add("Must be computed: \\\\ -in parallel \\\\ -without discretizing the surface")->at("constraints");
}