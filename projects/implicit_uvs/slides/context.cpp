#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateContextSlides(slope::Slideshow& show) {

    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame;

    show << Title("Context: Texturing implicit surfaces")->at(TOP);

    show << inNextFrame << PlaceLeft(Latex::Add("Implicit representations are a powerful tool to\\\\ represent shapes."));
    show << PlaceRelative(Image::Add("bunny_sdf.jpg",2.5),slope::ABS_RIGHT,slope::SAME_Y);

    show << Formula::Add("\\{f(x) = 0\\}")->at("impl_formula_1");

    show << newFrameSameTitle;

    show << PlaceRelative(Latex::Add("Perflecly fine representation of the geometry..."),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.2);
    show << inNextFrame << PlaceRelative(Latex::Add("...but what about textures?"),slope::ABS_LEFT,slope::REL_BOTTOM,0.1,0.04);

    show << inNextFrame << PlaceRelative(Latex::Add("Fundamental issue : implicit representations mostly \\\\ model volumes while textures are 2D objects!"),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.15);

    show << Image::Add("CSG.png")->at("csg");

    show << newFrameSameTitle;

    show << PlaceBelow(Latex::Add("Really hard to know your neighborhood..."));

    vecs X;
    X.push_back(vec::Zero());
    int N = 6;
    Faces F;
    float r = 0.7;
    float H = 0.4;
    for (size_t i = 0;i<N;i++) {
        float th = 2*M_PI*i/N;
        vec2 p = vec2(cos(th),sin(th))*r;
        float h = (p(0)*p(0) - p(1)*p(1))*H;
        X.push_back(vec(p(0),p(1),h));  
        slope::Face f{0,i+1,(i+1)%N+1};
        F.push_back(f);
    }
    auto Fan = Mesh::Add(X,F);
    auto verts = PointCloud::Add(X,0.03);

    vec offset = vec(2.5*r,0.,0);

    auto implicit = Context.disk->apply([offset,r,H](const vec& v){
        return vec(vec(v(0)*r,v(1)*r,H*(v(0)*v(0)-v(1)*v(1))) + offset);  
    });

    show << CameraView::Add("mesh_vs_impl");
    show << inNextFrame << Fan << verts << Latex::Add("Mesh")->at("mesh");

    auto X0 = PointCloud::Add({offset},0.03);
    show << inNextFrame << implicit << X0 << Latex::Add("Implicit")->at("impl");
    show << inNextFrame << Latex::Add("?")->track([offset](){return offset;},vec2(0.02,0.02));
    show << implicit->at(0.5);
}