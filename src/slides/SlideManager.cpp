#include "SlideManager.h"

slope::SlideManager::TransitionSets slope::SlideManager::computeTransitionsBetween(const Slide &A, const Slide &B)
{
    PrimitiveSet common,uniqueA,uniqueB;
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
    //sort by depth
    Primitives common_sorted(common.begin(),common.end());
    std::sort(common_sorted.begin(),common_sorted.end(),[](PrimitivePtr a,PrimitivePtr b){
        return a->getDepth() < b->getDepth();
    });
    Primitives uniqueA_sorted(uniqueA.begin(),uniqueA.end());
    std::sort(uniqueA_sorted.begin(),uniqueA_sorted.end(),[](PrimitivePtr a,PrimitivePtr b){
        return a->getDepth() < b->getDepth();
    });

    Primitives uniqueB_sorted(uniqueB.begin(),uniqueB.end());
    std::sort(uniqueB_sorted.begin(),uniqueB_sorted.end(),[](PrimitivePtr a,PrimitivePtr b){
        return a->getDepth() < b->getDepth();
    });

    return {common_sorted,uniqueA_sorted,uniqueB_sorted};
}

void slope::SlideManager::precomputeTransitions(){
    initialized = true;
    appearing_primitives.resize(slides.size());
    for (auto& p : slides[0])
        appearing_primitives[0].push_back(p.first);
    for (int i = 0;i<slides.size()-1;i++){
        transitions.push_back(
            computeTransitionsBetween(
                slides[i],
                slides[i+1]
                ));
        appearing_primitives[i+1] = std::get<2>(transitions.back());
    }
}
