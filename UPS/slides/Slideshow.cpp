#include "Slideshow.h"

void UPS::Slideshow::nextFrame()
{
    if (current_slide == slides.size()-1)
        return;
    current_slide++;
    fromClick = Time::now();
    std::cout << "[next frame] " << current_slide << std::endl;
}

void UPS::Slideshow::play() {
    auto window_flags = ImGuiConfig();
    ImGui::Begin("Unified Presentation System",NULL,window_flags);

    if (!transitions_computed)
        precomputeTransitions();

    auto t = TimeFrom(fromClick);

    auto& CS = slides[current_slide];

    if (current_slide > 0){
        auto& PS = slides[current_slide-1];
        auto&& [common,UA,UB] = transitions[current_slide-1];
        if (t < 2*transitionTime){
            for (auto c : common) {
                Primitive::get(c)->transition(0.5*t/transitionTime,t,
                                              PS[c],
                                              CS[c]
                                              );
            }
            if (t < transitionTime){
                for (auto ua : UA) {
                    Primitive::get(ua)->outro(t/transitionTime,PS[ua]);
                }
            }
            else {
                for (auto ub : UB){
                    Primitive::get(ub)->intro(t/transitionTime-1.,CS[ub]);
                }
                //slides[current_slide].intro(t/transitionTime-1.);
            }
        }
        else{
            for (auto& s : CS){
                Primitive::get(s.first)->draw(t - 2*transitionTime,CS[s.first]);
            }
        }
    }
    else {
        if (t < transitionTime)
            for (auto s : CS){
                std::cout << "intro" << std::endl;
                Primitive::get(s.first)->intro(t/transitionTime,CS[s.first]);
            }
        else
            for (auto s : CS)
                Primitive::get(s.first)->draw(t - transitionTime,CS[s.first]);
    }
    if (ImGui::IsKeyPressed(262)){
        nextFrame();
    }
    ImGui::End();
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

bool UPS::Slideshow::belongInSlide(const Slide &S, const PrimitiveID &id)
{
    for (auto s : S)
        if (s.first == id)
            return true;
    return false;
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
