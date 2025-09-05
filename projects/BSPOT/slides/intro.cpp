#include "intro.h"

using namespace slope;

void CreateIntroSlides(slope::Slideshow& show) {
    show << Title("BSP-OT: Sparse transport plans between \\\\ discrete measures in loglinear time")->at("title");
    show << Image::Add("teaser.jpg",0.7)->at("teaser");
    show << LatexLoader::LoadWithAnchor("authors");

    show << newFrame << Title("What is Optimal Transport (OT)?")->at(TOP);

    show << LatexLoader::LoadWithAnchor("ot_definition");

    show << inNextFrame << LatexLoader::LoadWithAnchor("ot_problem");
}
