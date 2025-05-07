#include "../implicit_uvs_slides.h" 

using namespace slope;

struct seed {
    vec p,e1,e2;

    seed(vec p,vec e1,vec e2) : p(p),e1(e1),e2(e2) {}

    seed(vec p,vec e1) : p(p),e1(e1) {
        e2 = e1.cross(vec(0,0,1));
    }

    vec2 operator()(const vec& x) const {
        vec d = x-p;
        return vec2(d.dot(e1),d.dot(e2));
    }
};



void CreateMultipleSeedsSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();
    show << newFrame;

    show << Title("One seed is not enough")->at(TOP);

    show << Image::Add("one_seed_bunny.png",0.6)->at(CENTER);

    show << PlaceAbove(Latex::Add("On smooth surfaces, our method provides Log maps possibly valid on a very large scale..."),0.01);    

    show << inNextFrame << Latex::Add("but, our heuristic has limits and Log maps accumulate distortion with curvature.")->at("one_seed");


    show << newFrame << Title("Multiple local fields")->at(TOP);
    auto multiple = Latex::Add("When multiple seeds are placed, we pick the uv of the closest seed with respect to the geodesic distance.");

    show << PlaceRelative(multiple,slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);
    show << Image::Add("multiple_seeds_bunny_single.png")->at("multiple_seeds_single");
    show << newFrameSameTitle << multiple;
    show << Latex::Add(tex::center("Implicit decals \n\n [de Groot et al. 2014]"))->at("impl_decals");
    show << PlaceBelow(Latex::Add("Euclidean distance",0.7));
    show << PlaceBottom(Image::Add("geodesic_voronoi_cells.png",0.5),0.5,0.2);
    show << Latex::Add("Ours")->at("ours 1");
    show << PlaceBelow(Latex::Add(tex::center("Geodesic distance \\\\ + segmentation"),0.7));
    show << newFrameSameTitle;
    show << PlaceBelow(Latex::Add("A lot of seams!"));
    //show << multiple;
    auto seams = Image::Add("seams.png",0.8);
    seams->setDepth(1);
    show << Image::Add("multiple_seeds_bunny.png",0.8)->at("multiple_seeds_bunny");
    show << Latex::Add(tex::center("Already enough to paint on the surface \\\\ but not pratical to apply structured textures."))->at("paint");
    show << inNextFrame << seams->at("multiple_seeds_bunny");
    /*
    show << newFrame << Title("Kernel techniques")->at(TOP);
    show << Image::Add("vector_kernel_diffusion.png")->at("kernel_techniques_img");
    show << Latex::Add("Since we have estimates for the geodesic distance \\\\ and parallel transport, we can use kernel techniques:")->at("kernel_techniques");
    show << PlaceBelow(Formula::Add("u_\\sigma(x) = \\frac{\\sum_{i=0}^{n} K_\\sigma(d_M(x,y_i)) R_{\\TM{y_i} \\rightarrow\\TM{x}}v_i}{\\sum_{i=0}^{n} K_\\sigma(d_M(x,y_i))}"));
    show << inNextFrame << PlaceBottom(Latex::Add("Could we apply such techniques to uv-fields?"));
    */
}
