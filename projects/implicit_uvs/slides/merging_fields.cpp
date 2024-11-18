
#include "../implicit_uvs_slides.h" 

using namespace slope;

#include <Eigen/Geometry>

struct Merging
{
    scalar sigma = 0.3;
    std::vector<vec> sources, e1, e2;
    vec E1,E2;

    Merging(int n) {
        sources.resize(n);
        e1.resize(n);
        e2.resize(n);
    }

    scalar dist_to_frontier(const vec &x, const vec &xi, const vec &xj)
    {
        scalar di = (x - xi).norm();
        scalar dj = (x - xj).norm();
        scalar dij = (xi - xj).norm();
        return (di * di - dj * dj) / (2 * dij);
    }

    scalar smoothstep(scalar x)
    {
        return x * x * (3 - 2 * x);
    }

    scalar wij(const vec &x, const vec &xi, const vec &xj)
    {
        scalar d = dist_to_frontier(x, xi, xj);
        if (d < -sigma)
            return 1;
        if (d > sigma)
            return 0;
        return smoothstep(0.5 - d / (2 * sigma));
    }

    scalar wi(const vec &x, int i)
    {
        scalar w = 1;
        for (int j = 0; j < sources.size(); j++)
        {
            if (i == j)
                continue;
            w = std::min(w, wij(x, sources[i], sources[j]));
        }
        return w;
    }

    vec Log(vec x, int i)
    {
        vec d = x - sources[i];
        return E1*d.dot(e1[i]) + E2*d.dot(e2[i]) + sources[i];
    }

    vec blended_field(const vec &x)
    {
        vec E = vec::Zero();
        float W = 0.;
        for (int i = 0; i < sources.size(); i++)
        {
            W += wi(x, i);
            E += wi(x, i) * Log(x, i);
        }
        return E / W;
    }
};

void CreateMergingFieldsSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame << Title("Merging uv-fields")->at(TOP);

    auto txt1 = Latex::Add("We wish to take weighted averages of multiple uv-fields.");
    show << PlaceRelative(txt1,slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.04);

//    show << Image::Add("merging_param_labeled.png",0.4)->at("merging_fields");

    vec p = vec(1,1,1).normalized();
    auto Ep = slope::CompleteBasis(p);


    show << CameraView::Add("merging_fields");

    scalar l = 0.8;
    vec ys[3];
    ys[0] = ExpSphere(p,Ep.first*l);
    ys[1] = ExpSphere(p,(Ep.first*cos(2*M_PI/3.) + Ep.second*sin(2*M_PI/3.))*l);
    ys[2] = ExpSphere(p,(Ep.first*cos(4.*M_PI/3.) + Ep.second*sin(4.*M_PI/3.))*l);

    vec zero = vec::Zero();
    auto Sp = Point::Add(zero,0.99);

    show << Sp << Point::Add(p) << Formula::Add("x")->at(p,vec2(0.02,-0.02));
    slope::Point::PointPtr ys_pts[3];

    auto E = slope::CompleteBasis(p);
    vec E1 = LogSphere(p,vec(0,0,1)).normalized();
    vec E2 = E1.cross(p);

    vec e1[3];
    vec e2[3];

    vec angles = vec(0.1,-0.1,0.2);


    for (int i = 0;i<3;i++){
        mat R = slope::SmallestRotation(p,ys[i]);
        mat rot = Eigen::AngleAxisd(angles(i),p).toRotationMatrix();
        e1[i] = R*rot*E1*0.4;
        e2[i] = R*rot*E2*0.4;
        ys_pts[i] =  Point::Add(ys[i],0.05);
        ys_pts[i]->pc->setPointColor(glm::vec3(1,1,0));
        auto _e1 = ys_pts[i]->addVector(e1[i]);
        _e1->q->setVectorColor(glm::vec3(1,0,0));
        auto _e2 = ys_pts[i]->addVector(e2[i]);
        _e2->q->setVectorColor(glm::vec3(0,0,1));
        show <<ys_pts[i] << _e1 << _e2;
        show << Formula::Add("y_" + std::to_string(i+1),Options::DefaultLatexScale*0.7)->at(ys[i],vec2(0.02,0.03));
    }
    show << inNextFrame;
    for (int i = 0;i<3;i++){
        vec log_x = LogSphere(ys[i],p)*0.5;
        show << ys_pts[i]->addVector(log_x);
    }
    show << Formula::Add("\\Log_{y_2}(x)",Options::DefaultLatexScale*0.7)->at("log_y1");
    auto txt2 = Latex::Add("However, each Log map is expressed in its own referential!");
    show << PlaceRelative(txt2,txt1,slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.01);
    show << inNextFrame;

    vec uv_offset = -LogSphere(p,ys[2]).normalized()*2.5;

    vec normal = E1.cross(E2);

    Merging M(3);
    M.E1 = E1;
    M.E2 = E2;
    slope::Mesh::MeshPtr uv_frames[3];

    int f0 = show.getNumberSlides() - 1;

    slope::DynamicParam get_source[3];

    scalar s = 0.6;
    LatexPtr refs[3];
    LatexPtr names[3];

    for (int i = 0;i<3;i++) {

        vec log = LogSphere(p,ys[i]);
        M.sources[i] = E1*log.dot(E1) + E2*log.dot(E2) + uv_offset;
        M.e1[i] = E1*cos(angles(i)) + E2*sin(angles(i));
        M.e2[i] = -E1*sin(angles(i)) + E2*cos(angles(i));

        vec b_pos = scalar(1.2*i+2)*E2;

        get_source[i] = [b_pos,i,M,f0,E2,s](TimeObject t) {
            if (t.absolute_frame_number == f0) {
                return b_pos;
            }
            if (t.absolute_frame_number == f0 + 1) {
                return lerp(b_pos,M.sources[i],smoothstep(t.from_action*s));
            }
            return M.sources[i];
        };

        auto get_e1_i = [f0,i,M,s] (TimeObject t) {
            if (t.absolute_frame_number == f0) {
                return M.E1;
            }
            if (t.absolute_frame_number == f0 + 1) {
                return slerp(M.E1,M.e1[i],smoothstep(t.from_action*s));
            }
            return M.e1[i];
        };
        auto get_e2_i = [f0,i,M,s] (TimeObject t) {
            if (t.absolute_frame_number == f0) {
                return M.E2;
            }
            if (t.absolute_frame_number == f0 + 1) {
                return slerp(M.E2,M.e2[i],smoothstep(t.from_action*s));
            }
            return M.e2[i];
        };


        uv_frames[i] = Context.grid->applyDynamic([get_e1_i,get_e2_i,i,M,normal,get_source](const Vertex& v,const TimeObject& t) {
            auto x = v.pos;
            vec p = (get_e1_i(t)*x(0) + get_e2_i(t)*x(1))*0.5  + get_source[i](t) + normal*0.01;
            return p;
        });
        uv_frames[i]->pc->setEdgeWidth(1);
        uv_frames[i]->pc->setSurfaceColor(glm::vec3(1,1,1));
        show << uv_frames[i]->at(0.9);
        auto pt =Point::Add(get_source[i],0.05); 
        auto pf1 = pt->addVector([get_e1_i](TimeObject t) {return vec(get_e1_i(t)*0.3);});
        auto pf2 = pt->addVector([get_e2_i](TimeObject t) {return vec(get_e2_i(t)*0.3);});
        pf1->q->setVectorColor(glm::vec3(1,0,0));
        pf2->q->setVectorColor(glm::vec3(0,0,1));
        pt->pc->setPointColor(glm::vec3(1,1,0));
        show << pt << pf1 << pf2;
        refs[i] = Formula::Add("\\left(0,0\\right)",Options::DefaultLatexScale*0.6);
        names[i] = Formula::Add("\\Log_{y_" + std::to_string(i+1)+"}",Options::DefaultLatexScale*0.8);
        show << refs[i]->at(b_pos,vec2(-0.02,0.02))<< names[i]->at(b_pos,vec2(0,0.15));
    }
    show << inNextFrame;
    show << PlaceRelative(Latex::Add("We have to express everything in a common uv-space."),txt2,slope::ABS_LEFT,slope::REL_BOTTOM,0.03,0.01);
    for (int i = 0;i<3;i++){
        show >> refs[i] >> names[i];
        show << Formula::Add("u_" + std::to_string(i+1),slope::Options::DefaultLatexScale*0.7)->at(M.sources[i],vec2(0.02,0.02));
    }
    show << Image::Add("referential.png",0.5)->at("uv_ref");

    auto grid_thin = Mesh::Add(Options::DataPath+"meshes/grid_quad_50.obj")->apply([E1,E2,normal,uv_offset](const vec& x){
        vec p = x(0)*E1 + x(1)*E2 + uv_offset;
        return p;
    });

    int n = 51;
    const auto& V = grid_thin->getVertices();
    std::vector<vec2> uvs(V.size(),vec2::Zero());
    for (int j = 0;j<n;j++)
        for (int i = 0;i<n;i++){
            vec p = M.blended_field(V[j*n+i]);
            uvs[j*n + i](0) = E1.dot(p);
            uvs[j*n + i](1) = E2.dot(p);
        }

    auto param = slope::AddPolyscopeQuantity<polyscope::SurfaceVertexParameterizationQuantity>(grid_thin->pc->addVertexParameterizationQuantity("blended",uvs));
    param->q->setStyle(polyscope::ParamVizStyle::GRID);
    param->q->setCheckerSize(0.1);
    param->q->setGridColors({glm::vec3(0,0,0),glm::vec3(1,1,1)});
    show << inNextFrame << grid_thin << param;
    show << Formula::Add("u(x) = \\frac{\\sum_i w_i(x) \\left( \\Log_{y_i}(x) + u_i \\right)}{\\sum_i w_i(x)}",Options::DefaultLatexScale*0.8)->at("merging_formula");
    for (int i = 0;i<3;i++)
        show >> uv_frames[i];

    show << newFrameSameTitle;

    auto sub = Latex::Add("Description of the workflow");
    show << PlaceBelow(sub);
    show <<  Gif::Add("workflow.gif",30,0.8)->at("workflow");
    show << PlaceRelative(Latex::Add("On the CPU, once: "),sub,slope::ABS_LEFT,slope::REL_BOTTOM,0.02,0.02);
    show << PlaceRelative(Latex::Add("- From the set of seeds, the user defines which seeds should be merged in a graph $A = (\\{y_i\\},E)$"),slope::ABS_LEFT,slope::REL_BOTTOM,0.05,0.02);
    show << PlaceRelative(Latex::Add("- Two seeds are merged if $(i,j) \\in E$, and we \\\\ compute the geodesic distance $d_{ij}$ between them."),slope::ABS_LEFT,slope::REL_BOTTOM,0.05,0.02);
    show << PlaceRelative(Latex::Add("- Compute optimal frames and offsets $u_i$"),slope::ABS_LEFT,slope::REL_BOTTOM,0.05,0.02);
    show << inNextFrame << PlaceRelative(Latex::Add("On the GPU, at each frame:"),slope::ABS_LEFT,slope::REL_BOTTOM,0.02,0.03);
    show << PlaceRelative(Latex::Add("- Find closest seed $i_0$. "),slope::ABS_LEFT,slope::REL_BOTTOM,0.05,0.02);
    show << PlaceRelative(Latex::Add("- Compute blending with the neighbors of $i_0$:"),slope::ABS_LEFT,slope::REL_BOTTOM,0.05,0.02);
    show << Formula::Add("u(x) = \\frac{\\sum_{i \\in N(i_0)} w_i(x) \\left( \\Log_{y_i}(x) + u_i \\right)}{\\sum_{i\\in N(i_0)} w_i(x)}")->at("merging_formula_graph");
}
