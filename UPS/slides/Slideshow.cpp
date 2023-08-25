#include "Slideshow.h"

void UPS::Slideshow::nextFrame()
{
    if (current_slide == slides.size()-1)
        return;
    current_slide++;
    fromAction = Time::now();
    backward = false;
    locked = true;
}

void UPS::Slideshow::previousFrame()
{
    if (!current_slide)
        return;
    for (auto& s : slides[current_slide])
        Primitive::get(s.first)->forceDisable();
    current_slide--;
    for (auto& s : slides[current_slide])
        Primitive::get(s.first)->intro(1.,s.second);
    fromAction = Time::now();
    backward = true;
    locked = true;
}

void UPS::Slideshow::forceNextFrame()
{
    if (current_slide == slides.size()-1)
        return;
    for (auto& s : slides[current_slide])
        Primitive::get(s.first)->forceDisable();
    current_slide++;
    for (auto& s : slides[current_slide])
        Primitive::get(s.first)->intro(1.,s.second);
    fromAction = Time::now();
    backward = false;
    locked = false;
}

void UPS::Slideshow::play() {
    auto window_flags = ImGuiConfig();
    ImGui::Begin("Unified Presentation System",NULL,window_flags);

    if (!transitions_computed)
        precomputeTransitions();

    auto t = TimeFrom(fromAction);
    auto tb = TimeFrom(fromBegin);

    auto& CS = slides[current_slide];

    if (backward || !locked) {
        locked = false;
        for (auto s : CS)
            Primitive::get(s.first)->play(tb,CS[s.first],current_slide);
    }
    else {
        if (current_slide > 0){
            auto& PS = slides[current_slide-1];
            auto&& [common,UA,UB] = transitions[current_slide-1];
            if (t < 2*transitionTime){
                for (auto c : common) {
                    auto st = transition(0.5*t/transitionTime,
                                         PS[c],
                                         CS[c]);
                    Primitive::get(c)->play(tb,st,current_slide);
                }
                if (t < transitionTime)
                    for (auto ua : UA)
                        Primitive::get(ua)->outro(t/transitionTime,PS[ua]);
                else
                    for (auto ub : UB)
                        Primitive::get(ub)->intro(t/transitionTime-1.,CS[ub]);
            }
            else {
                for (auto& s : CS)
                    Primitive::get(s.first)->play(tb,CS[s.first],current_slide);
                locked = false;
            }
        }
        else {
            if (t < transitionTime)
                for (auto s : CS)
                    Primitive::get(s.first)->intro(t/transitionTime,CS[s.first]);
            else {
                for (auto s : CS)
                    Primitive::get(s.first)->play(tb,CS[s.first],current_slide);
                locked = false;
            }
        }
    }

    if (ImGui::IsKeyPressed(262) && !locked){
        nextFrame();
    }
    else if (ImGui::IsKeyPressed(263)){
        previousFrame();
    }else if (ImGui::IsKeyPressed(264)){
        forceNextFrame();
    }
    ImGui::End();
}

UPS::StateInSlide UPS::Slideshow::transition(parameter t, const StateInSlide &sa, const StateInSlide &sb){
    StateInSlide St;
    St.relative_anchor_pos.x = std::lerp(sa.relative_anchor_pos.x,sb.relative_anchor_pos.x,smoothstep(t));
    St.relative_anchor_pos.y = std::lerp(sa.relative_anchor_pos.y,sb.relative_anchor_pos.y,smoothstep(t));
    St.alpha = std::lerp(sa.alpha,sb.alpha,smoothstep(t));
    St.angle = std::lerp(sa.angle,sb.angle,smoothstep(t));
    return St;
}

void UPS::Slideshow::precomputeTransitions()
{
    transitions_computed = true;
    for (int i = 0;i<slides.size()-1;i++){
        transitions.push_back(
            computeTransitionsBetween(
                slides[i],
                slides[i+1]
                ));
    }
}

UPS::Slideshow::TransitionSets UPS::Slideshow::computeTransitionsBetween(const Slide &A, const Slide &B)
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

ImGuiWindowFlags UPS::Slideshow::ImGuiConfig()
{
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGuiWindowFlags window_flags = 0;
    io.WantCaptureMouse = false;
    io.WantCaptureKeyboard = true;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoResize;
    window_flags |= ImGuiWindowFlags_NoBackground;
    window_flags |= ImGuiWindowFlags_NoScrollbar;

    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x,io.DisplaySize.y));
    return window_flags;
}


