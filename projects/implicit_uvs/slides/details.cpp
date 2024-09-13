#include "../implicit_uvs_slides.h" 

using namespace slope;

scalar sdf_circle(const vec& x) {
    return x.norm() - 1.;
}

vec proj(const vec& x) {
    return x.normalized();
}


scalar angle(const vec& x) {
    return std::atan2(-x(0),x(1));
}

scalar disp(scalar th) {
    return sin(th*10.)*0.1;
}

std::pair<Vec,Vec> load_fields(Mesh::MeshPtr Grid) {
    std::string cache1 = Options::DataPath + "cache/iuv_base_sdf.csv";
    std::string cache2 = Options::DataPath + "cache/iuv_displaced_sdf.csv";
    Vec base,displ;
    if (!io::VecCache(cache1,base)) {
        base = Grid->eval([](const vec& x) { return sdf_circle(x); });
        std::cout << "base not found in cache, recompute" << std::endl;
        io::SaveVec(cache1,base);
    }
    if (!io::VecCache(cache2,displ)) {
        displ = Grid->eval([](const vec& x) {
            if (x.norm() < 0.01)
                return -1.;
            return sdf_circle(x) - disp(angle(proj(x))); });
        std::cout << "displaced not found in cache, recompute" << std::endl;
        io::SaveVec(cache2,displ);
    }
    return {base,displ};
}

void CreateDetailsSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame << Title("Applications");
    show << newFrame << Title("Details on implicit surfaces")->at(TOP);
    show << PlaceBelow(Latex::Add("Color, roughness, normal maps"));

    show << PlaceRelative(Latex::Add("Applying textures is straightforward"),ABS_LEFT,REL_BOTTOM,0.02,0.1);

    show << Latex::Add("Using normal maps is also possible\\\\ using parallel transport:");
    show << Formula::Add("n(x) = R_{\\TM{y_i}\\rightarrow\\TM{x}}\\text{TBN}(y_i)n_{\\text{map}}(\\text{uv}(x))")->at("normal_map");

    show << Image::Add("robot.png",0.8)->at("robot");
    show << Gif::Add("eye_moving.gif",15,0.8)->at("mike");

    show << newFrameSameTitle << PlaceBelow(Latex::Add("Displacement maps"));

    show << Latex::Add("Displacement maps encode a displacement \\\\ of the surface in the normal direction.");

    show << inNextFrame << Latex::Add("It is trickier since it must take effect on the  \\\\ entire scalar field whereas the displacement \\\\ is only defined at the surface.");
    show << CameraView::Add("displacement");


    auto Grid = Context.grid200->scale(2.);
    Grid->pc->setEdgeWidth(0);

    int fn = show.getNumberSlides() - 1;

    vec q = vec(1.5,-1.5,0);


    auto&& F = load_fields(Grid);
    Vec base = F.first;Vec displaced = F.second;

    vec pq = proj(q);
    vec y = vec(1,0,0);

    auto dist = PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(Grid->pc->addVertexSignedDistanceQuantity("sdf",Vec::Zero(Grid->getVertices().size())));
    dist->q->setIsolineWidth(0.05,true);

    scalar r = 0.08;

    auto query = Point::Add([fn,q,pq](TimeObject t) {
        if (t.absolute_frame_number == fn)
            return q;
        if (t.absolute_frame_number == fn + 1)
            return lerp<vec>(q,pq,smoothstep(t.from_action));
        return pq;
    },r);

    Grid->updater = [Grid,dist,fn,base,displaced] (TimeObject t,Primitive*) {
        dist->q->updateData(Grid->eval([fn,t,base,displaced](const Vertex& v) {
            if (t.absolute_frame_number <= fn + 3) {
                return base[v.id];
            }
            if (t.absolute_frame_number == fn + 4) {
                scalar s = smoothstep(t.from_action);
                return base[v.id]*(1-s) + displaced[v.id]*s;
            }
            return displaced[v.id];
        }));
    };

    auto disp_f = Formula::Add("f_h = f - h \\circ \\text{uv} \\circ \\Pi_f");

    auto source = Point::Add(y,r);
    
    auto geo = [y,pq,fn](scalar t,TimeObject to) {
        if (to.absolute_frame_number == fn + 2)
            return slerp(pq,y,smoothstep(to.from_action)*t);
        return slerp(pq,y,t);
    };

    auto arc = Curve3D::Add(geo,100,false,0.03);

    show << Grid << query << source << dist;// << Formula::Add("y")->at(y,vec2(0.0,-0.02)) << Formula::Add("x")->at(q,vec2(0.02,-0.02));

    show << inNextFrame << Latex::Add("To overcome such difficulty, we first \\\\ project the query point on the surface:");
    show << Formula::Add("\\Pi_f(x) = x - f(x) \\nabla f(x)")->at("proj_f");
    
    show << inNextFrame << Latex::Add("We get:") << disp_f->at("disp_formula") << inNextFrame << arc << inNextFrame >> query >> arc >> source;

    show << newFrameSameTitle << PlaceBelow(Latex::Add("Displacement maps"));

    show << Latex::Add(tex::center("Can be used to add \\\\ details on a complex shape:"))->at("details");
    show << Image::Add("bunny_hair.png",1.)->at("bunny_hair");
    show << inNextFrame << Latex::Add("pretty expensive")->at("expensive");
    show << inNextFrame << Latex::Add(tex::center("Or to transform a simple shape \\\\ into a complex one!"))->at("morphing");
    show << Gif::Add("shape_conversion.gif",30,0.8)->at("shape_convert");
    show << inNextFrame << Latex::Add("super fast!")->at("super_fast");
    show << newFrame << Title("Still implicit!")->at(TOP);
    show << Gif::Add("implicit_advantages.gif",30,1.2)->at("advantages2");
    show << Latex::Add("We keep all the benefits of the implicit representation!\\\\(Warping, CSG, etc...)")->at("benefits");
}