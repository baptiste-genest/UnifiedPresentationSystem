#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"

namespace UPS {

class Slideshow
{
public:

    Slideshow() {
        fromAction = Time::now();
        fromBegin = Time::now();
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


    inline Slide& getCurrentSlide(){return slides.back();}
    inline Slide& getSlide(index i){return slides[i];}
    void duplicateLastSlide(){slides.push_back(slides.back());}

    void addToLastSlide(PrimitivePtr ptr,const StateInSlide& sis) {
        transitions_computed = false;
        if (slides.empty())
            slides.push_back(Slide());
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


    inline Slideshow& operator<<(std::pair<PrimitivePtr,StateInSlide> obj) {
        addToLastSlide(obj.first,obj.second);
        return *this;
    }

    inline Slideshow& operator<<(PrimitivePtr ptr) {
        addToLastSlide(ptr,{});
        return *this;
    }

    inline TimeObject getTimeObject() const {
        return TimeObject(TimeFrom(fromBegin),TimeFrom(fromAction),current_slide);
    }

private:

    bool backward = false;
    bool locked = true;

    static StateInSlide transition(parameter t,const StateInSlide& sa,const StateInSlide& sb);

    bool transitions_computed = true;

    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;
    TransitionSets computeTransitionsBetween(const Slide& A,const Slide& B);
    void precomputeTransitions();

    static ImGuiWindowFlags ImGuiConfig();

    TimeStamp fromAction,fromBegin;
    std::vector<Slide> slides;
    std::vector<TransitionSets> transitions;
    size_t current_slide = 0;
};

constexpr Slideshow::in_next_frame inNextFrame;
constexpr Slideshow::new_frame newFrame;

}

#endif // SLIDESHOW_H
