#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateIntroSlides(slope::Slideshow& show) {

    //show << Title(tex::center("Research Internship: \\\\ Exit Talk"))->at("title");
    show << Title(tex::center("Implicit UVs: Real time semi-global parameterization \\\\ of implicit surfaces"))->at("title");
    show << PlaceBelow(Latex::Add(tex::center("\\textbf{Baptiste Genest}, Pierre Gueth, Jérémy Levallois, Stephanie Wang"),Options::DefaultLatexScale*0.7),0.05);

//    show << Latex::Add("\\color{red} CONFIDENTIAL")->at("confidential");

    show << Image::Add("Adobe-Logo.png",0.1f)->at("adobe");

    //show << newFrame <<
}
