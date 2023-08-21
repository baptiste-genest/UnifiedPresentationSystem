#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"

namespace UPS {

class Slideshow
{
public:
    Slideshow() {fromClick = Time::now();}

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

private:

    bool transitions_computed = true;

    void precomputeTransitions();
    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;

    TransitionSets computeTransitionsBetween(const Slide& A,const Slide& B);


    static bool belongInSlide(const Slide& S,const PrimitiveID& id);

    static ImGuiWindowFlags ImGuiConfig();

    TimeStamp fromClick;
    std::vector<Slide> slides;
    std::vector<TransitionSets> transitions;
    size_t current_slide = 0;
};

}

#endif // SLIDESHOW_H
