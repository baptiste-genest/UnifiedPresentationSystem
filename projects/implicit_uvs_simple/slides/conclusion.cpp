#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateConclusionSlides(slope::Slideshow& show){

    show << newFrame << Title("Much more!")->at(TOP);
    show << PlaceBelow(Latex::Add("Implicit shell maps"));

    //show << Gif::Add("sampling.gif",15,1.,true)->at("sampling");
    show << Gif::Add("shell_space_bunny.gif",15,0.8,true)->at("shell_space");
    show << Gif::Add("shell_maps_virus.gif",15,0.9,true)->at("shell_space_2");
    show << newFrame << Title("Conclusion")->at(TOP);

    show << LatexLoader::Load("conclusion")->at("conclusion");
    show << Image::Add("EG.png",1.)->at("EG");
    show << inNextFrame << LatexLoader::Load("appli")->at("appli");
    show << inNextFrame << LatexLoader::Load("limitations")->at("limits");
    show << inNextFrame << LatexLoader::Load("futurwork")->at("future");
    //show << Latex::Add("We introduce the first method to compute uv coordinates \\\\ on implicit surfaces without discretization, in real time.")->at("summary");

    show << newFrame << Title("Thank you! \\\\ Questions?")->at(TOP);
    show << Gif::Add("texture_seat.gif",20,1.3,false)->at("texture_seat");
    show << PlaceBelow(Latex::Add(tex::center("Simple texturing example."),slope::Options::DefaultLatexScale*0.6));
}
