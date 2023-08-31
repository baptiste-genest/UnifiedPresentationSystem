#include "CameraView.h"

UPS::CameraView::CameraViewPtr UPS::CameraView::Add(const vec& f,const vec& t)
{
    CameraViewPtr rslt = NewPrimitive<CameraView>();
    rslt->from = glm::vec3(f(0),f(1),f(2));
    rslt->to = glm::vec3(t(0),t(1),t(2));
    return rslt;
}
