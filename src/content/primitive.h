#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "../UPS.h"
#include "../math/kernels.h"
#include "TimeObject.h"
#include "Options.h"
#include <fstream>
#include <iostream>
#include "io.h"

namespace UPS {

//Make it into a class to force read pos from label if non null
struct StateInSlide {
    Vec2 relative_anchor_pos = {0.,0.};
    scalar alpha = 1,angle=0;
    std::string label = "";

    StateInSlide() {}
    StateInSlide(const Vec2& p) : relative_anchor_pos(p) {}

    void setLabel(std::string label);
    void writeAtLabel(double x,double y,bool overwrite);

    Vec2 getAbsoluteAnchorPos() const {
        auto S = ImGui::GetWindowSize();
        return Vec2(relative_anchor_pos.x*S.x,relative_anchor_pos.y*S.y);
    }

    void readFromLabel();
};

using PrimitiveInSlide = std::pair<PrimitivePtr,StateInSlide>;

struct Primitive {

    using Size = ImVec2;
    using Updater = std::function<void(TimeObject,Primitive*)>;

    Updater updater = [] (TimeObject,Primitive*) {};

    PrimitiveID pid;
    static std::vector<PrimitivePtr> primitives;

    virtual bool isScreenSpace() {return true;}

    static void addPrimitive(PrimitivePtr ptr) {
        ptr->pid = primitives.size();
        primitives.push_back(ptr);
    }

    static PrimitivePtr get(PrimitiveID id){
        return primitives[id];
    }
    template<class T>
    static std::shared_ptr<T> get(PrimitiveID id){
        return std::static_pointer_cast<T>(primitives[id]);
    }

    inline std::pair<PrimitivePtr,StateInSlide> at(const StateInSlide& sis) {
        return {get(pid),sis};
    }
    inline std::pair<PrimitivePtr,StateInSlide> at(scalar x,scalar y) {
        return {get(pid),ImVec2(x,y)};
    }
    inline std::pair<PrimitivePtr,StateInSlide> at(scalar alpha) {
        StateInSlide sis;
        sis.alpha = alpha;
        return {get(pid),sis};
    }

    inline std::pair<PrimitivePtr,StateInSlide> at(std::string label) {
        StateInSlide sis;
        sis.setLabel(label);
        return {get(pid),sis};
    }

    std::set<index> visited_slides;
    index relativeSlideIndex(index in) {
        visited_slides.insert(in);
        return std::distance(visited_slides.begin(),visited_slides.find(in));
    }

    void handleInnerTime() {
        inner_time = Time::now();
    }

    void play(const TimeObject& t,const StateInSlide& sis) {
        auto it = t(this);
        draw(it,sis);
        updater(it,this);
    }

    virtual void draw(const TimeObject& time,const StateInSlide& sis) = 0;
    virtual void intro(const TimeObject& t,const StateInSlide& sis) = 0;
    virtual void outro(const TimeObject& t,const StateInSlide& sis) = 0;
    virtual void forceDisable() {};
    virtual void forceEnable() {};
    virtual Size getSize() const {return Size();}
    virtual void initPolyscope() {}

    Size getRelativeSize() const {
        auto S = Vec2(Options::UPS_screen_resolution_x, UPS::Options::UPS_screen_resolution_y);
        auto s = getSize();
        return Vec2(s.x/S.x,s.y/S.y);
    }

    TimeTypeSec getInnerTime(){
        return TimeFrom(inner_time);
    }


    bool exclusive = false;
    bool isExclusive() const {
        return exclusive;
    }
private:
    TimeStamp inner_time;
};

template<class T>
static std::shared_ptr<T> DuplicatePrimitive(std::shared_ptr<T> ptr){
    auto other = std::make_shared<T>(*ptr);
    Primitive::addPrimitive(other);
    other->initPolyscope();
    return other;
}

template <class T,typename... Args>
std::shared_ptr<T> NewPrimitive(Args&& ... args){
    auto ptr = std::make_shared<T>(std::forward<Args>(args)...);
    Primitive::addPrimitive(ptr);
    ptr->initPolyscope();
    return ptr;
}


}

#endif // PRIMITIVE_H
