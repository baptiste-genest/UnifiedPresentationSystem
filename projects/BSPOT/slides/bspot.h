#ifndef BSPOT_H
#define BSPOT_H

#include "../../../src/slope.h"

using namespace slope;

using plan = std::vector<int>;
using pts2D = std::vector<vec2>;

polyscope::CurveNetwork* plotBijection(const pts2D& X,const pts2D& Y,const plan& T);


struct BSPOTSlides {

    static BSPOTSlides& getContext(){
        static BSPOTSlides* context;
        if (!context) context = new BSPOTSlides();
        return *context;
    }
};

#endif // BSPOT_H
