#ifndef TRANSITIONS_H
#define TRANSITIONS_H

#include "StateInSlide.h"
#include "TimeObject.h"

namespace slope {

using TransitionAction = std::function<StateInSlide(const TimeObject&,const StateInSlide&)>;

struct TransitionAnimator {
    TransitionAction intro,outro;
    TransitionAnimator();
};

TransitionAnimator FadeInFadeOut();
TransitionAnimator SlideInSlideOut();

}

#endif // TRANSITIONS_H
