#include "../implicit_uvs_slides.h" 

using namespace slope;


void CreateMultipleSeedsSlides(slope::Slideshow& show) {
    auto Context = ImplicitUVsSlides::getContext();
    show << newFrame;

    show << Title("One seed is not enough")->at(TOP);

    show << Image::Add("one_seed_bunny.png",0.6);

    show << PlaceAbove(Latex::Add("On smooth surfaces, our method provides Log maps possibly valid on a very large scale..."),0.01);    

    show << inNextFrame << Latex::Add("but, our heuristic has limits and Log maps accumulate distortion with curvature.")->at("one_seed");

    show << newFrame << Title("Handling multiple seeds")->at(TOP);

    show << PlaceRelative(Latex::Add("When multiple seeds are placed, the uv at a point is the one provided by the closest seed..."),slope::ABS_LEFT,slope::REL_BOTTOM,0.04,0.05);
    show << inNextFrame << Latex::Add("with respect to the \\emph{geodesic} distance! (+ segmentation)")->at("geo_voronoï");

    show << inNextFrame << PlaceBottom(Image::Add("geodesic_voronoi_cells.png",0.6),0.5,0.2);
    show << Latex::Add(tex::center("Implicit decals \\\\ de Groot et al. 2014"))->at("impl_decals");
    show << Latex::Add("ours")->at("ours 1");

    show << newFrame << Title("Multiple local fields")->at(TOP);
    show << Image::Add("multiple_seeds_bunny.png",0.8)->at("multiple_seeds_bunny");
    show << Latex::Add(tex::center("Already enough to paint on the surface \\\\ but not pratical to apply structured textures."))->at("paint");
    show << newFrame << Title("Kernel techniques")->at(TOP);
    show << Image::Add("vector_kernel_diffusion.png")->at("kernel_techniques_img");
    show << Latex::Add("Since we have estimates for the geodesic distance \\\\ and parallel transport, we can use kernel techniques:")->at("kernel_techniques");
    show << PlaceBelow(Formula::Add("u_\\sigma(x) = \\frac{\\sum_{i=0}^{n} K_\\sigma(d_M(x,y_i)) R_{\\TM{y_i} \\rightarrow\\TM{x}}v_i}{\\sum_{i=0}^{n} K_\\sigma(d_M(x,y_i))}"));
    show << inNextFrame << PlaceBottom(Latex::Add("Could we apply such techniques to uv-fields?"));
}