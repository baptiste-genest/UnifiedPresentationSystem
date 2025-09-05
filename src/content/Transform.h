#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "libslope.h"
#include <glm/gtx/matrix_interpolation.hpp>

namespace slope {

using TransformMat = glm::mat4;

class Transform {
public:
    float angle;
    glm::vec3 axis,translation,scale;

    Transform() {
        angle = 0;
        scale = glm::vec3(1,1,1);
        axis = glm::vec3(0,0,1);
        translation = glm::vec3(0,0,0);
    }

    inline TransformMat getRotation() const {
        return glm::axisAngleMatrix(axis,angle);
    }

    TransformMat getMatrix() const {
        TransformMat rslt = glm::scale(TransformMat(1.f),scale);
        rslt = getRotation()*rslt;
        rslt[3] = glm::vec4(translation,1);
        return rslt;
    }

    static Transform Interpolate(const Transform& T1,const Transform& T2,scalar t) {
        Transform T;
        auto R1 = T1.getRotation();
        auto R2 = T2.getRotation();
        auto R = glm::interpolate(R1,R2,(float)t);
        glm::axisAngle(R,T.axis,T.angle);
        T.translation = T1.translation*float(1-t) + T2.translation*float(t);
        T.scale = T1.scale*float(1-t) + T2.scale*float(t);
        return T;
    }

    static Transform Translation(const vec& x) {
        Transform T;
        T.translation.x = x(0);
        T.translation.y = x(1);
        T.translation.z = x(2);
        return T;
    }

    static Transform AxisAngle(scalar th, vec x) {
        Transform T;
        T.angle = th;
        x.normalize();
        T.axis.x = x(0);
        T.axis.y = x(1);
        T.axis.z = x(2);
        return T;
    }

    static Transform Scale(const vec& x) {
        Transform T;
        T.scale.x = x(0);
        T.scale.y = x(1);
        T.scale.z = x(2);
        return T;
    }

    static Transform Scale(scalar x) {
        Transform T;
        T.scale.x = x;
        T.scale.y = x;
        T.scale.z = x;
        return T;
    }

    static Transform ScalePositionRotate(const vec& s,const vec& p,vec axis,scalar th) {
        Transform T;
        T.scale.x = s(0);
        T.scale.y = s(1);
        T.scale.z = s(2);
        T.translation.x = p(0);
        T.translation.y = p(1);
        T.translation.z = p(2);
        T.angle = th;
        axis.normalize();
        T.axis.x = axis(0);
        T.axis.y = axis(1);
        T.axis.z = axis(2);
        return T;
    }

    static Transform ScalePositionRotate(scalar s,const vec& p,const vec& axis,scalar th) {
        Transform T;
        T.scale.x = s;
        T.scale.y = s;
        T.scale.z = s;
        T.translation.x = p(0);
        T.translation.y = p(1);
        T.translation.z = p(2);
        T.angle = th;
        T.axis.x = axis(0);
        T.axis.y = axis(1);
        T.axis.z = axis(2);
        return T;
    }


};

}


#endif // TRANSFORM_H
