#ifndef PARAMETRIZATION_H
#define PARAMETRIZATION_H

#include "../UPS.h"

namespace UPS {

struct Parametrization {
    curve_param param;

    Parametrization(curve_param&& p) : param(p) {}
    Parametrization(const curve_param& p) : param(p) {}

    Parametrization(){
        param = [] (scalar t) {return vec::Ones();};
    }

    Parametrization operator+(const vec& v) const {
        Parametrization rslt;
        auto p = param;
        rslt.param = [p,v] (scalar t) {
            return vec(p(t)+v);
        };
        return rslt;
    }
    Parametrization operator*(scalar l) const {
        Parametrization rslt;
        auto p = param;
        rslt.param = [p,l] (scalar t) {
            return vec(p(t)*l);
        };
        return rslt;
    }

    vec operator()(scalar t) const {
        return param(t);
    }

    operator curve_param () const {
        return param;
    }

    void operator=(const curve_param& p) {
        param = p;
    }

};


}
#endif // PARAMETRIZATION_H
