#include <polyscope/polyscope.h>
#include "../../src/UnifiedPresentationSystem.h"


UPS::Slideshow show;


UPS::vec f_scale(UPS::vec x) {
  return x*0.5;
}

int main(int argc,char** argv)
{
  //Init
  show.init("tutorials",argc,argv);
  
  //Slideshow title
  show << UPS::Latex::Add("Hello 3D World!",UPS::Options::UPS_TITLE)->at(UPS::CENTER);
  
  //New slide
  auto title = UPS::Title("The Bunny")->at(UPS::TOP);
  show << UPS::newFrame << title;
  
  //Loading a 3d object
  auto bunny = UPS::Mesh::Add(UPS::Options::DataPath + "meshes/bunny.obj");
  show << bunny;
  
  // Adding the bunny again but scaled down
  show <<UPS::newFrame 
       << UPS::Title("The Bunny (scaled down when loaded)")->at(UPS::TOP)
       << UPS::Mesh::Add(UPS::Options::DataPath + "meshes/bunny.obj", 0.5);
  
  //Using apply()
  //Note that a new instance of the object is created
  show <<UPS::newFrame << UPS::Title("The Bunny (apply)")->at(UPS::TOP)
       << bunny->apply(f_scale,false);
  
  auto lambda_scale = [](const UPS::vec &v) { return UPS::vec(v*0.5 + UPS::vec(1.,0.,0.));};
  show << bunny->apply(lambda_scale,false);
  
  
  show <<UPS::newFrame << UPS::Title("The Bunny with quantities")->at(UPS::TOP) ;
  auto bunnyred =  bunny->apply(lambda_scale,false);
  auto lambda_x = [](const UPS::vec &v) { return sin(v(0));};
  auto vals = bunnyred->eval(lambda_x);
  bunnyred->setSmooth(true);
  show << bunnyred;
  show << UPS::AddPolyscopeQuantity( bunnyred->pc->addVertexScalarQuantity("posX", vals)->setColorMap("jet") ) ;
  show << UPS::Latex::Add("$\\sqrt{2}$");
  show << UPS::PlaceBelow(UPS::Latex::Add("$\\sqrt{3}$"));
  show << UPS::PlaceBelow(UPS::Latex::Add("$\\sqrt{5}$"));

  polyscope::state::userCallback = [](){
    show.play();
  };
  polyscope::show();
  return 0;
}

