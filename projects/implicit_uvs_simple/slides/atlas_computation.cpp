#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateAtlasComputeSlides(slope::Slideshow& show) {
    show << newFrame << Title("Optimal frames and uv-offsets")->at(TOP);

    show << Formula::Add("u(x) = \\frac{\\sum_i w_i(x) \\left( \\Log_{y_i}(x) + \\textcolor{red}{u_i} \\right)}{\\sum_i w_i(x)}",Options::DefaultLatexScale*2)->at(CENTER);

    vec center = vec(0,0,0);
    auto sphere = Point::Add(center,1);

    show << newFrameSameTitle << sphere;

    show << CameraView::Add("atlas");

    vec x0 = vec(0,0,1);
    vecs x(3);
    slope::Point::PointPtr pts[3];
    for (int i = 0;i<3;i++) {
        scalar th = 2*M_PI*i/3.;
        x[i] = ExpSphere(x0,vec(cos(th),sin(th),0)*0.4);
        pts[i] = Point::Add(x[i],0.05);
        show << pts[i]; 
    }
    slope::Curve3D::Curve3DPtr geos[3];
    geos[0] = Curve3D::Add([x](scalar t){
        return ExpSphere(x[0],t*LogSphere(x[0],x[1]));
    });
    geos[1] = Curve3D::Add([x](scalar t){
        return ExpSphere(x[1],t*LogSphere(x[1],x[2]));
    });
    geos[2] = Curve3D::Add([x](scalar t){
        return ExpSphere(x[2],t*LogSphere(x[2],x[0]));
    });
    for (int i = 0;i<3;i++){
        show << geos[i];
        geos[i]->pc->setColor(glm::vec3(1,1,0));
    }

    show << inNextFrame << PlaceRelative(Latex::Add("First, we compute the frames by minimizing a similarity energy: "),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);
    auto e_frame = Formula::Add("\\min_{t_i} \\sum_{(i,j) \\in E} \\frac{||R_{i\\rightarrow j} t_i - t_j||^2}{d_{ij}^2}");
    show << e_frame->at("frame_energy");

    vec t[3];
    t[0] = (LogSphere(x[0],x[1]) + LogSphere(x[0],x[2])).normalized()*0.3;
    t[1] = SmallestRotation(x[0],x[1])*t[0];
    t[2] = SmallestRotation(x[0],x[2])*t[0];
    vec t1 = -(LogSphere(x[1],x[0]) + t[1]).normalized()*0.3;
    vec t2 = -(LogSphere(x[2],x[0]) + t[2]).normalized()*0.3;
    show << pts[0]->addVector(t[0]);
    show << pts[1]->addVector(t1);
    show << pts[2]->addVector(t2);
    show << Formula::Add("t_i")->at(vec(x[0] + t[0]),vec2(0.02,-0.02));
    show << Formula::Add("t_j")->at(vec(x[1] + t1),vec2(0.02,-0.02));
    //show << Formula::Add("t_2")->at(vec(x[2] + t2),vec2(0.02,-0.02));

    show << inNextFrame;
    for (int i = 0;i<3;i++){
        auto t_i = pts[i]->addVector(t[i]);
        t_i->q->setVectorColor(glm::vec3(0.5,0.5,1));
        show << t_i;
    }
    show << Formula::Add("R_{i\\rightarrow j}t_i")->at(vec(x[1] + t[1]),vec2(0.02,-0.02));
    show << inNextFrame;
    auto diff = Curve3D::Add([x,t,t1](scalar s){
        vec v = (1-s)*(x[1] + t[1]) + s*(x[1] + t1);
        return v;
    });
    diff->pc->setColor(glm::vec3(1,0,0));
    show << diff;
    vec rt0 = x[0].cross(t[0]);
    show << inNextFrame << pts[0]->addVector(rt0);

    show << Formula::Add("n(y_i) \\times t_i")->at(x[0] + rt0,vec2(0.02,-0.02));
    show << Latex::Add("Once the $t_i$ are computed, the frames \\\\  are completed by a 90° rotation.")->at("90_rotation");
    show << inNextFrame << PlaceRelative(Latex::Add("Then, we compute the uv-offsets by trying\\\\ to respect each referential as much as possible: "),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);
    show << Formula::Add("\\min_{u_i} \\sum_{(i,j) \\in E} \\frac{||\\Log_{y_i}(y_j) - u_j - u_i||^2}{d_{ij}^2}")->at("offset_energy");

    /*
    show  << PlaceRelative(Latex::Add("In order to minimize distortion, we compute optimal frames and\\\\ uv-offsets of each seeds by minimizing quadratic energies: "),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);
    show << Latex::Add("Frame energy:")->at("frame_energy_txt");
    show << Latex::Add("Offset energy:")->at("offset_energy_txt");
    auto e_frame = Formula::Add("\\min_{t_i} \\sum_{(i,j) \\in E} \\frac{||R_{i\\rightarrow j} t_i - t_j||^2}{d_{ij}^2}");
    show << e_frame->at("frame_energy");

    vec t[3];
    t[0] = (LogSphere(x[0],x[1]) + LogSphere(x[0],x[2])).normalized()*0.3;
    t[1] = SmallestRotation(x[0],x[1])*t[0];
    t[2] = SmallestRotation(x[0],x[2])*t[0];
    vec t1 = -(LogSphere(x[1],x[0]) + t[1]).normalized()*0.3;
    vec t2 = -(LogSphere(x[2],x[0]) + t[2]).normalized()*0.3;
    show << pts[0]->addVector(t[0]);
    show << pts[1]->addVector(t1);
    show << pts[2]->addVector(t2);
    show << Formula::Add("t_i")->at(vec(x[0] + t[0]),vec2(0.02,-0.02));
    show << Formula::Add("t_j")->at(vec(x[1] + t1),vec2(0.02,-0.02));
    //show << Formula::Add("t_2")->at(vec(x[2] + t2),vec2(0.02,-0.02));

    for (int i = 0;i<3;i++){
        auto t_i = pts[i]->addVector(t[i]);
        t_i->q->setVectorColor(glm::vec3(0.5,0.5,1));
        show << t_i;
    }
    show << Formula::Add("R_{i\\rightarrow j}t_i")->at(vec(x[1] + t[1]),vec2(0.02,-0.02));
    auto diff = Curve3D::Add([x,t,t1](scalar s){
        vec v = (1-s)*(x[1] + t[1]) + s*(x[1] + t1);
        return v;
    });
    diff->pc->setColor(glm::vec3(1,0,0));
    show << diff;
    vec rt0 = x[0].cross(t[0]);
    show << pts[0]->addVector(rt0);
    
    show << Formula::Add("n(y_i) \\times t_i")->at(x[0] + rt0,vec2(0.02,-0.02));
    show << Formula::Add("\\min_{u_i} \\sum_{(i,j) \\in E} \\frac{||\\Log_{y_i}(y_j) - u_j - u_i||^2}{d_{ij}^2}")->at("offset_energy");
*/

    show << newFrameSameTitle << PlaceBelow(Latex::Add("Solving small linear systems"));
    show << PlaceRelative(Latex::Add("Solving least-square problems $\\implies$ solving SPD linear systems "),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);
    show << inNextFrame << PlaceRelative(Latex::Add("Of the size of the number of seeds \\\\ in the graph \\textbf{rarely more than 10!}."),slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);
    show << Latex::Add("Instantly solvable on the CPU!")->at("instant_solve");
    show << Latex::Add("Recomputed only \\\\ when graph changes")->at("recompute_SPD");
    show << Gif::Add("graph_editing.gif",15)->at("graph_editing");
    show << Latex::Add(tex::center("Real time editing \\\\ of the uv-field."),slope::Options::DefaultLatexScale*0.6)->at("legend");
}
