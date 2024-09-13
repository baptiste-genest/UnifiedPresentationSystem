#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateContextSlides(slope::Slideshow& show) {

    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame;

    show << Title("Implicit surfaces")->at(TOP);

    show << PlaceRelative(Latex::Add("Implicit representations are a powerful tool to\\\\ represent shapes."),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.1);
    show << PlaceRight(Image::Add("bunny_sdf.jpg",1.8),0.3,0.04);

    show << Image::Add("CSG.png",0.7)->at("csg");

    show << Formula::Add("M = \\{f(x) = 0\\}")->at("impl_formula_1");

    show << Latex::Add("Many advantages : geometric modeling,\\\\ shape optimization, etc...")->at("use_cases");

    show << newFrame <<  Title("Visualizing Implicit surfaces")->at(TOP);
    show << PlaceBelow(Latex::Add("Context of the work"),0.01);
    
    auto T = slope::Mesh::Add(Options::DataPath+"meshes/torus.obj",1,true);
    T->pc->setEdgeWidth(0);

    show << T;
    show << CameraView::Add("sphere_tracing");

    show << Latex::Add("Sphere tracing is an algorithm to compute the intersection between a ray and the surface without \\emph{any} discretization.")->at("sphere_tracing_desc");


    vec p = vec(2,0,1.5);
    vec target = vec(-0.5,0,0);
    vec d = (target-p).normalized();

    auto pt = Point::Add(p,0.05);
    auto dir = pt->addVector(vec(d*0.4));

    show << pt << dir;
    auto cam = Image::Add("eye.png",0.5)->at("eye");
    cam.second.angle = 0.3;
    show << cam;


    vec ref = vec(-1,0.,0.5);

    auto E = slope::CompleteBasis(d);
    auto set_screen = [p,E] (vec x) {
        x += vec(0.1,0.1,0.);
        return vec(x(0)*E.first + x(1)*E.second + p);
    };

    auto screen = Context.grid->apply(set_screen);
    show << screen->at(0.5);
    screen->pc->setEdgeWidth(1);

    auto ray = [p,target](scalar t,const TimeObject &time) {
        if (time.relative_frame_number == 0)
            return vec(p + (target-p)*t*smoothstep(time.from_action*0.4));
        return vec(p + (target-p)*t);
    };

    auto ray_param = Curve3D::Add(ray);
    show << inNextFrame << ray_param;

    show << Point::Add(target) << Formula::Add("x \\text{ s.t. } f(x) = 0")->at(target,vec2(0.1,-0.02));

    show << inNextFrame << Latex::Add("To remain in this implicit setting,\\\\ everything must be: \\\\ - point-wise/parallel \\\\ - lightweight \\\\- \\textbf{without meshing the surface}")->at("constraints");


    show << newFrame << Title("Texturing implicit surfaces")->at(TOP);

    show << PlaceRelative(Latex::Add("Perflecly fine representation of the geometry..."),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.2);
    show << PlaceRelative(Latex::Add("...but what about textures?"),slope::ABS_LEFT,slope::REL_BOTTOM,0.1,0.04);


    show << Image::Add("UVMapping.png")->at("uv_map");

    show << inNextFrame << PlaceRelative(Latex::Add("Fundamental issue : implicit representations are defined in a volumetric way whereas textures are 2D objects!"),slope::ABS_LEFT,slope::ABS_BOTTOM,0.04,0.2);

    show << newFrameSameTitle;
    show << PlaceBelow(Latex::Add("Not explicitly 2D"));


    vecs X;
    int N = 6;
    Faces F;
    float r = 0.7;
    vec offset = vec(1.8*r,0.,0);

    X.push_back(-offset);

    float H = 0.45;
    for (size_t i = 0;i<N;i++) {
        float th = 2*M_PI*i/N;
        vec2 p = vec2(cos(th),sin(th))*r;
        float h = (p(0)*p(0) - p(1)*p(1))*H;
        X.push_back(vec(p(0),p(1),h) - offset);  
        slope::Face f{0,i+1,(i+1)%N+1};
        F.push_back(f);
    }
    auto Fan = Mesh::Add(X,F);
    auto verts = PointCloud::Add(X,0.03);


    auto implicit = Context.disk->apply([offset,r,H](const vec& v){
        return vec(vec(v(0)*r,v(1)*r,H*(v(0)*v(0)-v(1)*v(1))) + offset);  
    });

    show << CameraView::Add("mesh_vs_impl");
    show << Fan << verts << Latex::Add("Mesh")->at("mesh");
    show << PlaceRelative(Latex::Add(tex::center("The 2D nature of the object is \\\\ described by the connectivity \\\\ of the faces")),slope::SAME_X,slope::ABS_BOTTOM,0.,0.1);

    show << inNextFrame;

    show << implicit << Latex::Add(tex::center("Implicit \\\\ surface"))->at("impl");
    show << PlaceRelative(Latex::Add(tex::center("In practice, \\\\ purely volumetric")),slope::SAME_X,slope::ABS_BOTTOM,0.,0.1);
    show << inNextFrame; 
    auto volume =  Mesh::Add(Options::DataPath+"meshes/cube_simple.obj")->apply([offset,r](const vec& x) {
        vec p = x;
        p = offset + p*r;
        return p;
    });
    volume->setSmooth(false);
    volume->pc->setEdgeWidth(1.5);
    volume->pc->setSurfaceColor(glm::vec3(0.7,0.7,1.));
    show << volume->at(0.2);
}