#include "../implicit_uvs_slides.h" 

using namespace slope;

void CreateIntroSlides(slope::Slideshow& show) {

    show << Title(tex::center("Research Internship: \\\\ Exit Talk"))->at("title");
    show << PlaceBelow(Latex::Add(tex::center("Baptiste GENEST")),0.2);
    show << PlaceBelow(Latex::Add(tex::center("Pierre Gueth, Jeremy Levallois, Stephanie Wang"),Options::Slope_default_height_ratio*0.7),0.05);
    show << PlaceBelow(Latex::Add(tex::center("Subject : Details on implicit surfaces"),Options::Slope_default_height_ratio*0.7),0.05);


    show << Image::Add("Adobe-Logo.png",0.1f)->at("adobe");

    //show << newFrame << 
}