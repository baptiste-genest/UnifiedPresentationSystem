#ifndef SLIDEMANAGER_H
#define SLIDEMANAGER_H

#include "Slide.h"
#include "../math/utils.h"
#include "../content/PrimitiveGroup.h"
#include "../content/Placement.h"
#include "../content/PrimitiveGroup.h"
//#include "Panel.h"

namespace UPS {

inline StateInSlide transition(parameter t, const StateInSlide &sa, const StateInSlide &sb){
    StateInSlide St;
    St.setOffset(lerp(sa.getPosition(),sb.getPosition(),smoothstep(t)));
    St.alpha = std::lerp(sa.alpha,sb.alpha,smoothstep(t));
    St.angle = std::lerp(sa.angle,sb.angle,smoothstep(t));
    return St;
}
inline vec2 computeOffsetToMean(const Slide& buffer) {
    double x_min = 100,y_min = 100,x_max = 0,y_max = 0;
    for (auto&& [ptr,sis] : buffer) {
        auto pos = sis.getPosition();
        auto size = std::dynamic_pointer_cast<ScreenPrimitive>(ptr)->getRelativeSize();
        x_min = std::min(x_min,pos(0)-size(0)*0.5f);
        y_min = std::min(y_min,pos(1)-size(1)*0.5f);
        x_max = std::max(x_max,pos(0)+size(0)*0.5f);
        y_max = std::max(y_max,pos(1)+size(1)*0.5f);
    }
    vec2 bbox = vec2(x_max-x_min,y_max-y_min);
    return bbox*0.5f;
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

    ScreenPrimitivePtr last_screen_primitive_inserted,centering_root;

    Slide center_buffer;int center_start,center_end;
    bool centering = false;AnchorPtr center_anchor;

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
        if (ptr->isScreenSpace()){
            last_screen_primitive_inserted = std::static_pointer_cast<ScreenPrimitive>(ptr);
            if (centering){
                if (center_buffer.empty())
                    centering_root = last_screen_primitive_inserted;
                center_buffer.add(ptr,sis);
            }
        }
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
    struct in_next_frame{};
    struct new_frame{
        bool same_title = false;
    };

    void handleCenter() {
        if (!centering)
            return;
        centering = false;
        auto root_pos = computeOffsetToMean(center_buffer);
        auto old_pos = center_buffer[centering_root].getPosition();
        for (int i = center_start;i<getNumberSlides();++i)
            slides[i][centering_root].setOffset(-root_pos);
        center_buffer = Slide();
    }

    struct center_tag{bool open;};
    SlideManager& operator<<(center_tag ct) {
        if (ct.open){
            centering = true;
            center_start = getNumberSlides()-1;
        } else {
            handleCenter();
        }
        return *this;
    }
};

constexpr SlideManager::in_next_frame inNextFrame;
constexpr SlideManager::new_frame newFrame{false};
constexpr SlideManager::new_frame newFrameSameTitle{true};
constexpr SlideManager::center_tag beginCenter{true};
constexpr SlideManager::center_tag endCenter{false};


inline SlideManager& operator<<(SlideManager& SM,const Slide& S) {
    SM.addSlide(S);
    return SM;
}


inline SlideManager& operator<<(SlideManager& SM,SlideManager::in_next_frame) {
    SM.duplicateLastSlide();
    return SM;
}

inline SlideManager& operator<<(SlideManager& SM,SlideManager::new_frame nf) {
    SM.handleCenter();
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
