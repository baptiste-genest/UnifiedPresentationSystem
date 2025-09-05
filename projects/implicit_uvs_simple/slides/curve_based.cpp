#include "../implicit_uvs_slides.h" 

using namespace slope;


void CreateCurveBasedSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame << Title("Geometric information inferred along the curve")->at(TOP);

    auto PT = Latex::Add("Parallel transport");
    show << inNextFrame << PlaceBelow(PT,0.02);
    show << CameraView::Add("parallel_transport");

    show << PlaceRelative(Latex::Add("Parallel transport tells you how a tangent vector \\\\ is transformed when carried along a curve."),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);

    vec c = vec::Zero();
    auto sphere = Point::Add(c,1.);
    show << sphere;

    float l = 0.7;
    vec p1 = ExpSphere(vec(0,0,1),vec(l,0.,0.));
    vec p2 = ExpSphere(vec(0,0,1),vec(-l,0.,0.));

    auto ptx = Point::Add(p1,0.05);
    show << ptx;
    show << Formula::Add("x")->at(p1,vec2(0.03,0.03));
    show << Formula::Add("p")->at(p2,vec2(-0.03,0.03));
    auto pty = Point::Add(p2,0.05);
    show << pty;

    auto save = show.getCurrentSlide();

    vec Log = LogSphere(p1,p2);

    auto geo = [p1,p2,Log](scalar t,const TimeObject& time) {
        if (time.relative_frame_number == 0)
            return p1;
        if (time.relative_frame_number == 1)
            return ExpSphere(p1,t*Log*smoothstep(time.from_action));
        return ExpSphere(p1,t*Log);
    };

    auto Geo = Curve3D::Add(geo);

    auto geo_head = [Geo] (scalar) {
        return vec(Geo->getNodes().back());
    };

    auto carry_vector = Point::Add(geo_head,0.0);

    vec v = (Log + Log.cross(p1)).normalized()*0.3;

    auto transported_vector = [p1,carry_vector,v] (scalar t) {
        return vec(SmallestRotation(p1,carry_vector->getCurrentPos())*v);
    };
    show << Geo << carry_vector << carry_vector->addVector(transported_vector);

    show << inNextFrame << save;


    int N = 5;
    vecs iterates(N);
    for (int i = 0;i<N;i++)
        iterates[i] = ExpSphere(p1,Log*i/(N-1));

    auto iterates_pts = PointCloud::Add(iterates,0.03);
    auto normals = iterates_pts->pc->addVectorQuantity("n",iterates);
    normals->setVectorLengthScale(0.1);
    normals->setVectorRadius(0.01);
    show << iterates_pts << PolyscopeQuantity<polyscope::PointCloudVectorQuantity>::Add(normals);

    {
        vec x1 = iterates[1];
        vec x2 = iterates[2];
        vec Log = LogSphere(x1,x2);

        auto geo = [x1, x2, Log](const TimeObject &time)
        {
            if (time.relative_frame_number == 0)
                return x1;
            if (time.relative_frame_number == 1)
                return ExpSphere(x1, Log * smoothstep(time.from_action));
            return x2;
        };
        vec vp = SmallestRotation(p1,x1)*v;
        vec Tvp = SmallestRotation(x1,x2)*vp;
        auto tangent_mapped = [vp,Tvp] (const TimeObject& time) {
            if (time.relative_frame_number <= 1)
                return vp;
            if (time.relative_frame_number == 2)
                return vec(slerp(vp.normalized(),Tvp.normalized(),smoothstep(time.from_action))*.3);
            return Tvp;
        };
        auto normal_mapped = [x1,x2] (const TimeObject& time) {
            if (time.relative_frame_number <= 1)
                return vec(x1*.3);
            if (time.relative_frame_number == 2)
                return vec(slerp(x1,x2,smoothstep(time.from_action))*.3);
            return vec(x2*.3);
        };
        auto pt =Point::Add(geo,0.);
        show << pt << pt->addVector(normal_mapped) << pt->addVector(tangent_mapped) << inNextFrame;
//        show << Latex::Add("It can be discretized by aligning \\\\ consecutive normals along the curve, \\\\ using the smallest rotation \\\\ between $n(x_i)$ and $n(x_{i+1})$.")->at("discretization");
//        show << Formula::Add("R_{n(x_i)\\rightarrow n(x_{i+1})}")->at("smallest_rot");

        show << inNextFrame ;
        show << Latex::Add("Rotation matrix:")->at("rot_mat");
        show << Formula::Add("R_{x \\rightarrow p} : \\TM{x} \\rightarrow \\TM{p}")->at("rot_mat_f");

//        show << Formula::Add("R_{\\TM{x} \\rightarrow \\TM{y}} = \\prod_{i = 1}^{N-1} R_{n(x_i)\\rightarrow n(x_{i+1})}")->at("parallel_transport_mat");
        show << inNextFrame << ptx->addVector(v) << Formula::Add("v")->at(vec(p1+v),vec2(0.0,-0.02));
        vp = SmallestRotation(p1,p2)*v;
        show << inNextFrame << pty->addVector(vp) << Formula::Add("v'")->at(vec(p2+vp),vec2(0.01,-0.03));
    }

    show << newFrameSameTitle << PlaceBelow(Latex::Add("Detection of sharp features"),0.02);

    show << inNextFrame << PlaceRelative(Latex::Add("Our method cannot cross sharp features in a robust way."),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);
    show << inNextFrame << PlaceRelative(Latex::Add("But we provide a way to detect \\\\ them to stop the computation:"),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);

    {
        auto base = show.getCurrentSlide();
        show << CameraView::Add("sharp_features");
        auto cube = Context.cube->copy();
        cube->pc->setEdgeWidth(0.);
        cube->pc->setSmoothShade(false);
        show << cube;

        vec x1 = vec(0.04, 1., 0.5);
        vec x2 = vec(0, 0.96, 0.5);
        auto p1 = Point::Add(x1, 0.03);
        auto p2 = Point::Add(x2, 0.03);

        vec n1 = vec(0., 1., 0.) * 0.3;
        vec n2 = vec(-1., 0., 0.) * 0.3;

        show << p1 << p1->addVector(n1) << p2 << p2->addVector(n2);
        // display labels x_{i} n(x_{i}) and x_{i+1} n(x_{i+1})
        show << Formula::Add("x_i")->at(x1, vec2(0.03, -0.02));
        show << Formula::Add("x_{i+1}")->at(x2, vec2(0.02, +0.04));
        show << Formula::Add("n(x_i)")->at(x1 + n1, vec2(0.04, 0.));
        show << Formula::Add("n(x_{i+1})")->at(x2 + n2, vec2(0., -0.04));

        auto arc = [x1,x2,n1,n2](scalar t,const TimeObject& time)
        {
            scalar r = 0.4;
            vec p;
            if (time.relative_frame_number == 0)
                p = (x1+x2)*0.5 + slerp(n1,n2,t*smoothstep(time.from_action))*r;
            else
                p = (x1+x2)*0.5 + slerp(n1,n2,t)*r;
            return p;
        };

        show << inNextFrame << Curve3D::Add(arc,100,false,0.01) << Formula::Add("\\theta")->at("sharp angle");

        auto crit = Latex::Add(" Stop if : $\\theta > \\epsilon$")->at("sharp_features_test");
        show << inNextFrame << crit ;

        show << base << crit << Latex::Add("This implies that the surface is \\\\ implicitly segmented into smooth regions.")->at("sharp_features_text");
        show << Image::Add("sharp_features.png",0.7)->at("sharp_features_img");
    }

    show << newFrame << Title("Everything is \\textit{realtime!}")->at(TOP);

    show << Latex::Add("Since everything is recomputed at each frame, per pixel, one can change \\emph{any} parameter and see the result immediately.")->at("advantages");

    show << inNextFrame << Gif::Add("uv_on_moving.gif",30,0.6)->at("uv_on_moving");
    show << PlaceBelow(Latex::Add(tex::center("change of shape \\\\ parameter"),slope::Options::DefaultLatexScale*0.6));
    show << Gif::Add("uv_on_noise.gif",30,0.8)->at("uv_on_noise");
    show << PlaceBelow(Latex::Add(tex::center("time dependant noise"),slope::Options::DefaultLatexScale*0.6));
    show <<  Gif::Add("move_seed_crop.gif",30,0.8)->at("move_seed");
    show << PlaceBelow(Latex::Add(tex::center("seed is moved"),slope::Options::DefaultLatexScale*0.6));
}
