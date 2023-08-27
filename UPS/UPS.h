#ifndef UPS_H
#define UPS_H

#include <string>

#include <vector>
#include <map>
#include <set>
#include <memory>

#include <cmath>
#include <iostream>

#include <chrono>
#include <ratio>
#include <functional>

#include "eigen3/Eigen/Dense"

#include "polyscope/polyscope.h"


namespace UPS {

class Widget;
class StateInSlide;

using WidgetPtr = std::shared_ptr<Widget>;

using Vec2 = ImVec2;
using RGBA = ImColor;


using Time = std::chrono::high_resolution_clock;
using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
using TimeTypeSec = float;
using DurationSec = std::chrono::duration<TimeTypeSec>;

inline TimeTypeSec TimeBetween(const TimeStamp& A,const TimeStamp& B){
    return DurationSec(B-A).count();
}

inline TimeTypeSec TimeFrom(const TimeStamp& A){
    return DurationSec(Time::now()-A).count();
}


using parameter = double;


using Animation = std::function<void(TimeTypeSec,const StateInSlide&)>;

using index = size_t;

using scalar = float;
constexpr scalar TAU = 2*M_PI;
using vec = Eigen::Vector<scalar,3>;
using vecs = std::vector<vec>;

using Face = std::vector<size_t>;
using Faces = std::vector<Face>;

using FontID = int;
using FontSize = int;
struct Font {
    FontID id = -1;
    FontSize size = 16;
};

struct Primitive;
using PrimitivePtr = std::shared_ptr<Primitive>;
using PrimitiveID = size_t;
using Primitives = std::set<PrimitiveID>;

#define UPS_prefix std::string("/home/eulerson314/dev/UnifiedPresentationSystem/data/")
#define UPS_screen_resolution Vec2(1920,1080);

using param = std::function<vec(scalar)>;
using mapping = std::function<vec(const vec&)>;

}

#endif // UPS_H
