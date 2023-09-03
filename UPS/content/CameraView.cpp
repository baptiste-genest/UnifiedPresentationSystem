#include "CameraView.h"

UPS::CameraView::CameraViewPtr UPS::CameraView::Add(const vec& f,const vec& t, const vec &up,bool flyTo)
{
    return NewPrimitive<CameraView>(toVec3(f),toVec3(t),toVec3(up),flyTo);
}



namespace UPS {
CameraView::CameraView(const glm::vec3 &from, const glm::vec3 &to, const glm::vec3 &up, bool flyTo) : from(from),
    to(to),
    up(up),
    flyTo(flyTo)
{}

}
