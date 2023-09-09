#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

#include "polyscope/polyscope.h"
#include "primitive.h"

namespace UPS {

class CameraView : public Primitive
{
public:
    static glm::vec3 toVec3(const vec& x) {
        return glm::vec3(x(0),x(1),x(2));
    }
    CameraView(const glm::vec3 &from, const glm::vec3 &to, const glm::vec3 &up,bool flyTo = false);

    CameraView() {}
    using CameraViewPtr = std::shared_ptr<CameraView>;
    static CameraViewPtr Add(const vec& from,const vec& to,const vec& up = vec(0,1,0),bool flyTo = false);

    // Primitive interface
public:
    void draw(const TimeObject &time, const StateInSlide &sis) override {}
    void intro(const TimeObject&, const StateInSlide &) override {    }
    void outro(const TimeObject&, const StateInSlide &sis) override {    }
    virtual void forceEnable() override {
        polyscope::view::lookAt(from,to,up,flyTo);
    }
    void forceDisable() override {
        polyscope::view::resetCameraToHomeView();
    }

private:
    glm::vec3 from,to,up;
    bool flyTo;
};



}

#endif // CAMERAVIEW_H
