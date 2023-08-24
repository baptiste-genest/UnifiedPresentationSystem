#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include "../UPS.h"
#include "../math/kernels.h"

namespace UPS {

struct StateInSlide {
    Vec2 relative_anchor_pos = {0.,0.};
    scalar alpha = 1;

    StateInSlide() {}
    StateInSlide(const Vec2& p) : relative_anchor_pos(p) {}

    Vec2 getAbsoluteAnchorPos() const {
        auto S = ImGui::GetWindowSize();
        return Vec2(relative_anchor_pos.x*S.x,relative_anchor_pos.y*S.y);
    }
};


struct Primitive {

    using Updater = std::function<void(TimeTypeSec,int,PrimitiveID)>;

    Updater updater = [] (TimeTypeSec,int,PrimitiveID) {};

    PrimitiveID pid;
    static std::vector<PrimitivePtr> primitives;

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
        return {get(pid),ImVec2{x,y}};
    }

    std::set<index> visited_slides;
    index relativeSlideIndex(index in) {
        visited_slides.insert(in);
        return std::distance(visited_slides.begin(),visited_slides.find(in));
    }

    void play(TimeTypeSec ts,const StateInSlide& sis,int absolute_slide_nb) {
        draw(ts,sis);
        updater(ts,relativeSlideIndex(absolute_slide_nb),pid);
    }

    virtual void draw(TimeTypeSec ts,const StateInSlide& sis) = 0;
    virtual void intro(parameter t,const StateInSlide& sis) = 0;
    virtual void outro(parameter t,const StateInSlide& sis) = 0;
    virtual void transition(parameter t,TimeTypeSec ts,const StateInSlide& sa,const StateInSlide& sb){
        StateInSlide St;
        St.relative_anchor_pos.x = std::lerp(sa.relative_anchor_pos.x,sb.relative_anchor_pos.x,smoothstep(t));
        St.relative_anchor_pos.y = std::lerp(sa.relative_anchor_pos.y,sb.relative_anchor_pos.y,smoothstep(t));
        St.alpha = std::lerp(sa.alpha,sb.alpha,t);
        draw(ts,St);
    }
};

template <class T>
std::shared_ptr<T> NewPrimitive(){
    auto ptr = std::make_shared<T>();
    Primitive::addPrimitive(ptr);
    return ptr;
}

}

#endif // PRIMITIVE_H
