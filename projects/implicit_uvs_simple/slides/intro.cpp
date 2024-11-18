#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateIntroSlides(slope::Slideshow& show) {

    show << Title(tex::center("Research Internship: \\\\ Exit Talk"))->at("title");
    show << PlaceBelow(Latex::Add(tex::center("Baptiste GENEST")),0.2);
    show << PlaceBelow(Latex::Add(tex::center("Pierre Gueth, Jérémy Levallois, Stephanie Wang"),Options::DefaultLatexScale*0.7),0.05);
    //show << PlaceBelow(Latex::Add(tex::center("Subject : Real-time parameterization of implicit surfaces"),Options::Slope_default_height_ratio*0.7),0.05);
    show << PlaceBelow(Latex::Add(tex::center("Real-time parameterizations and details on implicit surfaces"),Options::DefaultLatexScale*0.7),0.05);

//    show << Latex::Add("\\color{red} confidential",Options::Slope_default_height_ratio*0.6)->at("confidential");

    show << Image::Add("Adobe-Logo.png",0.1f)->at("adobe");

    //show << newFrame << 
}