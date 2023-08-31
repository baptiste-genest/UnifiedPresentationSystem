#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"
#include "../content/Placement.h"

namespace UPS {

class Slideshow
{
public:


    Slideshow(bool debug = false) : debug(debug) {
        from_action = Time::now();
        from_begin = Time::now();
    }

    void nextFrame();

    void previousFrame();

    void forceNextFrame();

    void play();

    void addSlide(const Slide& s) {
        transitions_computed = false;
        slides.push_back(s);
    }

    template<typename... T>
    void addSlides(const T& ... x) {
        (addSlide(x), ...);
    }

    TimeTypeSec transitionTime = 0.5;

    void setInnerTime();


    inline Slide& getCurrentSlide(){return slides.back();}
    inline Slide& getSlide(index i){return slides[i];}
    void duplicateLastSlide(){slides.push_back(slides.back());}

    void addToLastSlide(PrimitivePtr ptr,const StateInSlide& sis) {
        transitions_computed = false;
        if (slides.empty())
            slides.push_back(Slide());
        last_primitive_inserted = {ptr,sis};
        slides.back().add(ptr,sis);
    }

    struct in_next_frame{};
    struct new_frame{};

    inline Slideshow& operator<<(const Slide& S) {
        addSlide(S);
        return *this;
    }


    inline Slideshow& operator<<(in_next_frame) {
        duplicateLastSlide();
        return *this;
    }

    inline Slideshow& operator<<(new_frame) {
        slides.push_back(Slide());
        return *this;
    }
    
    inline Slideshow& operator<<(const RelativePlacement& P) {
        auto pos = P.computePlacement(last_primitive_inserted);
        addToLastSlide(P.ptr,StateInSlide(pos));
        return *this;
    }


    inline Slideshow& operator<<(PrimitiveInSlide obj) {
        addToLastSlide(obj.first,obj.second);
        return *this;
    }

    inline Slideshow& operator<<(PrimitivePtr ptr) {
        addToLastSlide(ptr,{StateInSlide(CENTER)});
        return *this;
    }

    inline TimeObject getTimeObject() const {
        return TimeObject{TimeFrom(from_begin),TimeFrom(from_action),(int)current_slide};
    }

    void handleTransition();

    void init();

private:

    PrimitiveInSlide last_primitive_inserted;

    bool transition_done = false;
    bool backward = false;
    bool locked = true;

    bool debug;

    static StateInSlide transition(parameter t,const StateInSlide& sa,const StateInSlide& sb);

    bool transitions_computed = true;

    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;
    Primitives& common(TransitionSets& S) {return std::get<0>(S);}
    Primitives& uniquePrevious(TransitionSets& S) {return std::get<1>(S);}
    Primitives& uniqueNext(TransitionSets& S) {return std::get<2>(S);}
    TransitionSets computeTransitionsBetween(const Slide& A,const Slide& B);
    void precomputeTransitions();

    static void ImGuiWindowConfig();

    int visited_slide = -1;

    TimeStamp from_action,from_begin;
    std::vector<Primitives> appearing_primitives;
    std::vector<Slide> slides;
    std::vector<TransitionSets> transitions;
    size_t current_slide = 0;
    ImGuiWindowFlags window_flags = 0;
};

constexpr Slideshow::in_next_frame inNextFrame;
constexpr Slideshow::new_frame newFrame;

}

#endif // SLIDESHOW_H
