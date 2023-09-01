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
    static CameraViewPtr Add(const vec& from,const vec& to,const vec& up = vec(0,1,0));

    // Primitive interface
public:
    void draw(const TimeObject &time, const StateInSlide &sis) override {}
    void intro(parameter, const StateInSlide &) override {    }
    void outro(parameter t, const StateInSlide &sis) override {    }
    virtual void forceEnable() override {
        polyscope::view::lookAt(from,to,up,true);
    }
    void forceDisable() override {
        polyscope::view::resetCameraToHomeView();
    }

    glm::vec3 from,to,up;
private:
};


}

#endif // CAMERAVIEW_H
