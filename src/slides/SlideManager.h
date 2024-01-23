#ifndef SLIDEMANAGER_H
#define SLIDEMANAGER_H

#include "Slide.h"
#include "../math/utils.h"
#include "../content/PrimitiveGroup.h"
#include "../content/Placement.h"
#include "../content/PrimitiveGroup.h"

namespace UPS {

inline StateInSlide transition(parameter t, const StateInSlide &sa, const StateInSlide &sb){
    StateInSlide St;
    St.setOffset(lerp(sa.getPosition(),sb.getPosition(),smoothstep(t)));
    St.alpha = std::lerp(sa.alpha,sb.alpha,smoothstep(t));
    St.angle = std::lerp(sa.angle,sb.angle,smoothstep(t));
    return St;
}

class SlideManager {
protected:

    std::vector<Slide> slides;
    bool transitions_computed = false;

    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;

    TransitionSets computeTransitionsBetween(const Slide &A, const Slide &B);

    std::vector<TransitionSets> transitions;

    Primitives& common(TransitionSets& S) {return std::get<0>(S);}
    Primitives& uniquePrevious(TransitionSets& S) {return std::get<1>(S);}
    Primitives& uniqueNext(TransitionSets& S) {return std::get<2>(S);}

    std::vector<Primitives> appearing_primitives;

    void precomputeTransitions();

    ScreenPrimitivePtr last_screen_primitive_inserted;

public:

    void duplicateLastSlide(){slides.push_back(slides.back());}

    void addSlide(const Slide& s) {
        transitions_computed = false;
        slides.push_back(s);
    }

    template<typename... S>
    void addSlides(const S& ... x) {
        (addSlide(x), ...);
    }

    inline int getNumberSlides() const {
        return slides.size();
    }

    inline Slide& getCurrentSlide(){return slides.back();}
    inline Slide& getSlide(index i){return slides[i];}

    void addToLastSlide(const PrimitiveInSlide& pis) {
        addToLastSlide(pis.first,pis.second);
    }

    void addToLastSlide(PrimitivePtr ptr,const StateInSlide& sis) {
        transitions_computed = false;
        if (slides.empty())
            slides.push_back(Slide());
        if (ptr->isScreenSpace())
            last_screen_primitive_inserted = std::static_pointer_cast<ScreenPrimitive>(ptr);
        slides.back().add(ptr,sis);
    }


    void removeFromCurrentSlide(PrimitivePtr ptr) {
        slides.back().remove(ptr);
    }

    void removeFromCurrentSlide(const PrimitiveGroup& G) {
        for (const auto& p : G.buffer)
            removeFromCurrentSlide(p.first);
    }

    Slide& getLastSlide() {
        return slides.back();
    }

    ScreenPrimitivePtr getLastScreenPrimitive() {
        return last_screen_primitive_inserted;
    }

};
struct in_next_frame{};
struct new_frame{
    bool same_title = false;
};

constexpr in_next_frame inNextFrame;
constexpr new_frame newFrame{false};
constexpr new_frame newFrameSameTitle{true};


inline SlideManager& operator<<(SlideManager& SM,const Slide& S) {
    SM.addSlide(S);
    return SM;
}


inline SlideManager& operator<<(SlideManager& SM,in_next_frame) {
    SM.duplicateLastSlide();
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,new_frame nf) {
    SM.addSlide(Slide());
    if (nf.same_title){
        int i = SM.getNumberSlides()-2;
        auto ex = SM.getSlide(i).exclusive_prim;
        if (ex != nullptr){
            SM.addToLastSlide(ex,SM.getSlide(i)[ex]);
        }
    }
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,const Replace& R) {
    PrimitiveInSlide old;
    if (!R.ptr_other)
        old = {SM.getLastScreenPrimitive(),SM.getLastSlide()[SM.getLastScreenPrimitive()]};
    else
        old = {R.ptr_other,SM.getLastSlide()[R.ptr_other]};
    auto pos = old.second;
    SM.removeFromCurrentSlide(old.first);
    SM.addToLastSlide(R.ptr,pos);
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,const RelativePlacement& P) {
    StateInSlide sis;
    if (!P.ptr_other)
        sis = P.computePlacement({SM.getLastScreenPrimitive(),SM.getLastSlide()[SM.getLastScreenPrimitive()]});
    else
        sis = P.computePlacement({P.ptr_other,SM.getLastSlide()[P.ptr_other]});
    SM.addToLastSlide(P.ptr,sis);
    return SM;
}


inline SlideManager& operator<<(SlideManager& SM,PrimitiveInSlide obj) {
    SM.addToLastSlide(obj.first,obj.second);
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,PrimitivePtr ptr) {
    SM.addToLastSlide(ptr,StateInSlide(vec2(0.5,0.5)));
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,const StateInSlide& sis) {
    SM.getLastSlide()[SM.getLastScreenPrimitive()] = sis;
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,const PrimitiveGroup& G) {
    for (auto [ptr,sis] : G.buffer){
        SM.addToLastSlide(ptr,sis);
    }
    return SM;
}


inline SlideManager& operator>>(SlideManager& SM,PrimitivePtr ptr) {
    SM.removeFromCurrentSlide(ptr);
    return SM;
}

    inline SlideManager& operator>>(SlideManager& SM,const PrimitiveGroup& G) {
        SM.removeFromCurrentSlide(G);
        return SM;
    }

}

#endif // SLIDEMANAGER_H
