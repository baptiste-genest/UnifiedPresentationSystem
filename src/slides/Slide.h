#ifndef SLIDE_H
#define SLIDE_H

#include "../content/primitive.h"
#include "../content/ScreenPrimitive.h"
#include "../content/CameraView.h"

namespace slope {

struct Transition{
    PrimitiveID pid;
    StateInSlide state_a;
    StateInSlide state_b;
};
using Transitions = std::vector<Transition>;

struct Slide : public std::map<PrimitivePtr,StateInSlide> {

    void add(PrimitivePtr p,const StateInSlide& sis = {}){
        if (p->isScreenSpace())
            if (p->isExclusive()){
                if (title_primitive != nullptr)
                    this->erase(title_primitive);
                title_primitive = std::static_pointer_cast<TextualPrimitive>(p);
            }
        (*this)[p] = sis;
    }
    void add(PrimitivePtr p,const vec2& pos){
        add(p,StateInSlide(pos));
    }

    std::vector<PrimitiveInSlide> getDepthSorted() {
        std::vector<PrimitiveInSlide> rslt;
        for (auto&& [ptr,sis] : *this)
            rslt.push_back({ptr,sis});
        std::sort(rslt.begin(),rslt.end(),[](PrimitiveInSlide a,PrimitiveInSlide b){
            return a.first->getDepth() < b.first->getDepth();
        });
        return rslt;
    }

    void add(PrimitiveInSlide pis){
        add(pis.first,pis.second);
    }

    void remove(PrimitivePtr ptr) {
        this->erase(ptr);
    }

    TextualPrimitivePtr title_primitive = nullptr;
    CameraViewPtr camera = nullptr;

    std::map<ScreenPrimitivePtr,StateInSlide> getScreenPrimitives() const {
        std::map<ScreenPrimitivePtr,StateInSlide> rslt;
        for (auto&& [ptr,sis] : *this) {
            if (!ptr->isScreenSpace())
                continue;
            rslt[std::dynamic_pointer_cast<ScreenPrimitive>(ptr)] = sis;
        }
        return rslt;
    }

    std::string getTitle() const {
        if (title_primitive == nullptr)
            return "";
        return title_primitive->content;
    }

    void setCam() const {
        if (camera)
            camera->enable();
        /*
        else
            polyscope::view::resetCameraToHomeView();
        */
    }

    bool sameCamera(const Slide& other) const {
        if (camera && !other.camera)
            return false;
        if (!camera && other.camera)
            return false;
        if (!camera && !other.camera)
            return true;
        return camera == other.camera;
    }


};

}

#endif // SLIDE_H
