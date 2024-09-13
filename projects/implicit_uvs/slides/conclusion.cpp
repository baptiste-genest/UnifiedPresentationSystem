#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateConclusionSlides(slope::Slideshow& show){

    show << newFrame << Title("Much more!")->at(TOP);

    show << Gif::Add("sampling.gif",15,1.,true)->at("sampling");
    show << PlaceBelow(Latex::Add("sampling"));
    show << Gif::Add("shell_space_bunny.gif",15,0.8,true)->at("shell_space");
    show << PlaceBelow(Latex::Add("Implicit shell maps"));
    show << newFrame << Title("Thank you! \\\\ Questions?")->at(TOP);
    show << Gif::Add("texture_seat.gif",20,1.5,false)->at("texture_seat");
}