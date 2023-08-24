#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"

namespace UPS {

class Slideshow
{
public:
    Slideshow() {fromAction = Time::now();}

    void nextFrame();

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


    inline Slide& getSlide(index i){return slides[i];}
    void duplicateLastSlide(){slides.push_back(slides.back());}

    void addToLastSlide(PrimitivePtr ptr,const StateInSlide& sis) {
        transitions_computed = false;
        if (slides.empty())
            slides.push_back(Slide());
        slides.back().add(ptr,sis);
    }

    inline Slideshow& operator<<(const Slide& S) {
        addSlide(S);
        return *this;
    }

    struct in_next_frame{};


    inline Slideshow& operator<<(in_next_frame) {
        duplicateLastSlide();
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

private:

    bool transitions_computed = true;

    void precomputeTransitions();
    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;

    TransitionSets computeTransitionsBetween(const Slide& A,const Slide& B);


    static bool belongInSlide(const Slide& S,const PrimitiveID& id);

    static ImGuiWindowFlags ImGuiConfig();

    TimeStamp fromAction;
    std::vector<Slide> slides;
    std::vector<TransitionSets> transitions;
    size_t current_slide = 0;
};

constexpr Slideshow::in_next_frame inNextFrame;
}

#endif // SLIDESHOW_H
