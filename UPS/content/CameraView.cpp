#include "CameraView.h"

UPS::CameraView::CameraViewPtr UPS::CameraView::Add(const vec& f,const vec& t, const vec &up)
{
    CameraViewPtr rslt = NewPrimitive<CameraView>();
    rslt->from = glm::vec3(f(0),f(1),f(2));
    rslt->to = glm::vec3(t(0),t(1),t(2));
    rslt->up = glm::vec3(up(0),up(1),up(2));
    return rslt;
}

