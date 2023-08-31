#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

#include "polyscope/polyscope.h"
#include "primitive.h"

namespace UPS {

class CameraView : public Primitive
{
public:
    using CameraViewPtr = std::shared_ptr<CameraView>;
    CameraView(){}
    static CameraViewPtr Add(const vec& from,const vec& to);

    // Primitive interface
public:
    void draw(const TimeObject &time, const StateInSlide &sis) override {}
    void intro(parameter, const StateInSlide &) override {
        if (!active){
            active = true;
            polyscope::view::lookAt(from,to,glm::vec3(0,0,1),true);
        }
    }
    void outro(parameter t, const StateInSlide &sis) override {
        if (active){
            active = false;
            polyscope::view::resetCameraToHomeView();
        }
    }
    void forceDisable() override {
        polyscope::view::resetCameraToHomeView();
        active = false;
    }

    glm::vec3 from,to;
private:
    bool active = false;
    polyscope::CameraParameters old;
};


}

#endif // CAMERAVIEW_H
