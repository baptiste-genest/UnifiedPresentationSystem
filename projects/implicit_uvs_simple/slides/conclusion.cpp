#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateConclusionSlides(slope::Slideshow& show){

    show << newFrame << Title("Much more!")->at(TOP);
//    show << PlaceBelow(Latex::Add("Implicit shell maps \\& tangent vector field interpolation"));

    //show << Gif::Add("sampling.gif",15,1.,true)->at("sampling");
//    show << Gif::Add("shell_space_bunny.gif",15,0.8,true)->at("shell_space");
//    show << PlaceRelative(Latex::Add("Extending 2D coordinates into a 3D shell."),placeX::ABS_LEFT,placeY::REL_BOTTOM,0.05,0.2);
//    show << Latex::Add("Extending 2D coordinates into a 3D shell.")->at("shell_def");
    show << Latex::Add(tex::center("Implicit shell maps: \\\\ Extending 2D coordinates into a 3D shell."))->at("shell_def");
    auto sm =  Gif::Add("shell_maps_virus.gif",15,0.75,true);
    sm->setDepth(-1);
    show << sm->at("shell_space_2");
    show << Image::Add("shell_map_figure.png")->at("shell_space");
    show << Latex::Add("Tangent vector field interpolation:")->at("VF_in_title");
    show << Image::Add("VF_interpolation.png",0.75)->at("VF_interp");
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
