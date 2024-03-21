#include "SlideManager.h"

UPS::SlideManager::TransitionSets UPS::SlideManager::computeTransitionsBetween(const Slide &A, const Slide &B)
{
    Primitives common,uniqueA,uniqueB;
    for (auto sb : B)
        uniqueB.insert(sb.first);

    for (auto sa : A){
        bool inB = false;
        for (auto sb : B){
            if (sa.first == sb.first){
                common.insert(sa.first);
                uniqueB.erase(sa.first);
                inB = true;
                break;
            }
        }
        if (!inB){
            uniqueA.insert(sa.first);
        }
    }
    return {common,uniqueA,uniqueB};
}

void UPS::SlideManager::precomputeTransitions(){
    initialized = true;
    appearing_primitives.resize(slides.size());
    for (auto& p : slides[0])
        appearing_primitives[0].insert(p.first);
    for (int i = 0;i<slides.size()-1;i++){
        transitions.push_back(
            computeTransitionsBetween(
                slides[i],
                slides[i+1]
                ));
        appearing_primitives[i+1] = std::get<2>(transitions.back());
    }
}
