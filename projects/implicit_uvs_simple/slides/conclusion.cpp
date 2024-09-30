#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateConclusionSlides(slope::Slideshow& show){

    show << newFrame << Title("Much more!")->at(TOP);
    show << PlaceBelow(Latex::Add("Implicit shell maps"));

    //show << Gif::Add("sampling.gif",15,1.,true)->at("sampling");
    show << Gif::Add("shell_space_bunny.gif",15,0.8,true)->at("shell_space");
    show << Gif::Add("shell_maps_virus.gif",15,0.9,true)->at("shell_space_2");
    show << newFrame << Title("Conclusion")->at(TOP);
    show << Latex::Add("Summary of contributions:\\\\\\\\- Good quality local uv-fields\\\\ on implicit surfaces\\\\\\\\- Merging uv-fields into\\\\ more global ones,\\\\ \\\\- Completely real-time/parallel,\\\\ keep implicit advantages.")->at("summary");
    show << Latex::Add("Outcomes : \\textbf{Eurographics 2025, patent}")->at("outcomes");
    show << Image::Add("EG.png",0.8)->at("EG");


    show << newFrame << Title("Thank you!")->at(TOP);
    show << Gif::Add("texture_seat.gif",20,1.3,false)->at("texture_seat");
    show << PlaceBelow(Latex::Add(tex::center("Simple texturing example."),slope::Options::Slope_default_height_ratio*0.6));

    show << Image::Add("batou2.jpg",0.7)->at("me");
    show << PlaceBelow(Latex::Add("mail : baptistegenest@gmail.com \\\\ website : baptiste-genest.github.io"),0.1);
}