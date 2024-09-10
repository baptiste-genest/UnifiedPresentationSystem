#include "../implicit_uvs_slides.h" 

using namespace slope;

scalar dVij(scalar d0,scalar d1,scalar d01) {
    return (d0*d0 - d1*d1)/(2*d01);
}

void CreateCompactSupportSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();

    show << newFrame << Title("Compact support interpolation")->at(TOP);
    auto sub = Latex::Add("Between two seeds");
    show << PlaceBelow(sub);

    auto base = show.getCurrentSlide();

    auto txt = Latex::Add("In order to reduce as much as possible the computations, we want to interpolate only around the frontier between two seeds.");
    show << PlaceRelative(txt,sub,slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.05);

    vec x1 = vec(-0.5,0.,0);
    vec x2 = vec(0.5,0.,0);

    auto p1 = Point::Add([x1] (scalar t) {
        t*= 2;
        return vec(x1 + vec(cos(t),sin(t),0)*0.3);
    });

    auto Grid = Context.grid30->copy();
    Grid->pc->setEdgeWidth(0);
    scalars D(Grid->getVertices().size(),0);
    auto dist = PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(Grid->pc->addVertexSignedDistanceQuantity("distance_frontier",D));

    Grid->updater = [Grid,dist,p1,x2] (TimeObject,Primitive*) {
        dist->q->updateData(Grid->eval([p1,x2](const vec& x) {
            scalar d0 = (x-p1->getCurrentPos()).norm();
            scalar d1 = (x-x2).norm();
            scalar d10 = (x2-p1->getCurrentPos()).norm();
            return dVij(d0,d1,d10);
        }));
    };



    show << inNextFrame << PlaceRelative(Latex::Add("In planar geometry, the signed distance to the Voronoï frontier between two points is given by:"),txt,slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.04);
    auto euc = Formula::Add("d_{V_{ij}}(x) = \\frac{||x-y_i||^2 - ||x-y_j||^2}{2||y_i-y_j||}");
    show << PlaceRelative(euc,slope::ABS_LEFT,slope::REL_BOTTOM,0.1,0.04);
    auto p2 = Point::Add(x2);
    show << p1 << p2;
    show << CameraView::Add("euc_dvij");

    show << Grid << dist;

    auto geo = Formula::Add("d^M_{V_{ij}}(x) \\approx \\frac{d^M(x,y_i)^2 - d^M(x,y_j)^2}{2d^M(y_i,y_j)}");


    show << inNextFrame >> p1 >> Grid >> p2 << PlaceRelative(Latex::Add("On surfaces, just use the geodesic distance instead!"),slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.04);
    show << PlaceRelative(geo,slope::ABS_LEFT,slope::REL_BOTTOM,0.1,0.04);

    auto Sp = Context.sphere->copy();
    Sp->pc->setEdgeWidth(0);
    scalars Ds(Sp->getVertices().size(),0);
    auto geo_dist = PolyscopeQuantity<polyscope::SurfaceVertexScalarQuantity>::Add(Sp->pc->addVertexSignedDistanceQuantity("distance_frontier",Ds));

    vec ref = vec(0,0,1);
    vec s2 = ExpSphere(ref,vec(0.5,0,0));
    vec s1 = ExpSphere(ref,-vec(0.5,0,0));
    auto ps1 = Point::Add([ref](scalar t){
        t*= 2;
        return ExpSphere(ref,vec(-0.5 + 0.3*cos(t),0.3*sin(t),0));
    });

    show << Point::Add(s2);
    show << ps1;

    Sp->updater = [Sp,geo_dist,ps1,s2] (TimeObject,Primitive*) {
        geo_dist->q->updateData(Sp->eval([ps1,s2](const vec& x) {
            scalar d0 = DistSphere(x,ps1->getCurrentPos());
            scalar d1 = DistSphere(x,s2);
            scalar d10 = DistSphere(ps1->getCurrentPos(),s2);
            return dVij(d0,d1,d10);
        }));
    };

    show << Sp << geo_dist;

    show << inNextFrame << 
    PlaceRelative(Latex::Add("We only want to blend in the band : $\\{|d^M_{V_{ij}}(x)| < \\sigma \\}$"),slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.01);

    show << base;

    show << PlaceRelative(Latex::Add("If we are far enough from the frontier, no blending!"),sub,slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.05);

    show << Formula::Add("w_{ij}(x) = ")->at("wij") << PlaceNextTo(Formula::Add(tex::cases("1","\\text{if } d^M_{V_{ij}}(x) < -\\sigma","\\omega(d^M_{V_{ij}})","\\text{if } -\\sigma < d^M_{V_{ij}}(x) < \\sigma","0","\\text{else}")),1);

    show << Image::Add("band_weight_labeled.png",0.35)->at("wij_formula");

    show << inNextFrame << Latex::Add("Since $d \\leq d^M$, there is a simply test to check if we can \\\\ avoid to compute $d^M(x,y_j)$:")->at("test_text");
    show << Formula::Add("d_M(x,y_i)^2 + 2 \\sigma d(y_i,y_j) < ||x-y_j||^2")->at("simple test");

    show << newFrameSameTitle << PlaceBelow(Latex::Add("Between n seeds"));
    show << PlaceRelative(Latex::Add("The weight for one seed $w_i(x)$ is the \\\\ minimum of its weights with its neighbors $w_{ij}(x)$:"),slope::ABS_LEFT,slope::REL_BOTTOM,0.01,0.05);
    show << Formula::Add("w_i(x) = \\min_{j | (i,j) \\in E} w_{ij}(x)")->at("wi_formula");
    show << Image::Add("wi.png")->at("wi_plot");

    show << Latex::Add("It can be seen as a smoothed indicatrix of the Geodesic Voronoï cell of $y_i$.")->at("voro_cell");
}