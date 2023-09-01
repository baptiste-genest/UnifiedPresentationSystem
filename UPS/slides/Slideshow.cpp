#include "Slideshow.h"

void UPS::Slideshow::nextFrame()
{
    if (current_slide == slides.size()-1)
        return;
    for (auto& s : uniqueNext(transitions[current_slide]))
        Primitive::get(s)->forceEnable();
    current_slide++;
    from_action = Time::now();
    backward = false;
    locked = true;
    transition_done = false;
}

void UPS::Slideshow::previousFrame()
{
    if (!current_slide)
        return;
    for (auto& s : uniqueNext(transitions[current_slide-1]))
        Primitive::get(s)->forceDisable();
    for (auto& s : uniquePrevious(transitions[current_slide-1]))
        Primitive::get(s)->forceEnable();
    current_slide--;
    for (auto& s : slides[current_slide]){
        auto p = Primitive::get(s.first);
        p->intro(1.,s.second);
    }
    from_action = Time::now();
    backward = true;
    locked = true;
}

void UPS::Slideshow::forceNextFrame()
{
    if (current_slide == slides.size()-1)
        return;
    for (auto& s : uniquePrevious(transitions[current_slide]))
        Primitive::get(s)->forceDisable();
    for (auto& s : uniqueNext(transitions[current_slide]))
        Primitive::get(s)->forceEnable();
    current_slide++;
    for (auto& s : slides[current_slide]){
        auto p = Primitive::get(s.first);
        p->intro(1.,s.second);
    }
    from_action = Time::now();
    backward = false;
    locked = false;
}

void UPS::Slideshow::play() {
    ImGuiWindowConfig();
    ImGui::Begin("Unified Presentation System",NULL,window_flags);

    if (!transitions_computed)
        precomputeTransitions();

    auto t = TimeFrom(from_action);

    auto& CS = slides[current_slide];

    TimeObject tobj = getTimeObject();

    setInnerTime();

    bool triggerIT = false;

    if (backward || !locked) {
        locked = false;
        for (auto s : CS)
            Primitive::get(s.first)->play(tobj,CS[s.first]);
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
                    Primitive::get(c)->play(tobj,st);
                }
                if (t < transitionTime)
                    for (auto ua : UA)
                        Primitive::get(ua)->outro(t/transitionTime,PS[ua]);
                else {
                    handleTransition();
                    for (auto ub : UB){
                        Primitive::get(ub)->intro(t/transitionTime-1.,CS[ub]);
                    }
                }
            }
            else {
                for (auto& s : CS)
                    Primitive::get(s.first)->play(tobj,CS[s.first]);
                locked = false;
            }
        }
        else {
            if (t < transitionTime){
                for (auto s : CS){
                    Primitive::get(s.first)->intro(t/transitionTime,CS[s.first]);
                }
            }
            else {
                for (auto s : CS)
                    Primitive::get(s.first)->play(tobj,CS[s.first]);
                locked = false;
            }
        }
    }

    prompt();

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

void UPS::Slideshow::setInnerTime()
{
    if (visited_slide == current_slide)
        return;
    for (auto& p : appearing_primitives[current_slide])
        Primitive::get(p)->handleInnerTime();
    visited_slide = current_slide;
}

void UPS::Slideshow::prompt()
{
    if (prompter_ptr == nullptr)
        return;
    for (const auto& R : scripts_ranges)
        if (R.inRange(current_slide)){
            prompter_ptr->write(R.tag,from_begin);
            return;
        }
    prompter_ptr->erase(from_begin);
}

void UPS::Slideshow::handleTransition()
{
    if (transition_done || current_slide == 0)
        return;
    transition_done = true;
    for (auto& s : std::get<1>(transitions[current_slide-1]))
        Primitive::get(s)->forceDisable();
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
    if (prompter_ptr != nullptr)
        prompter_ptr->loadScript();
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
    if (debug)
        current_slide = slides.size()-1;
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

void UPS::Slideshow::ImGuiWindowConfig()
{
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.WantCaptureMouse = false;
    io.WantCaptureKeyboard = true;
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x,io.DisplaySize.y));
}

void UPS::Slideshow::init(std::string script_file)
{
    if (!script_file.empty())
        setScriptFile(script_file);
    polyscope::init();
    polyscope::options::buildGui = false;
    polyscope::options::autocenterStructures = false;
    polyscope::options::autoscaleStructures = false;
    polyscope::options::groundPlaneEnabled = false;
    polyscope::options::giveFocusOnShow = false;
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::options::transparencyMode = polyscope::TransparencyMode::Pretty;
    polyscope::state::lengthScale = 2.;
    polyscope::state::boundingBox =
        std::tuple<glm::vec3, glm::vec3>{ {-1., -1., -1.}, {1., 1., 1.} };
    polyscope::view::upDir = polyscope::view::UpDir::ZUp;
    window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoResize;
    window_flags |= ImGuiWindowFlags_NoBackground;
    window_flags |= ImGuiWindowFlags_NoScrollbar;

}

void UPS::Slideshow::setScriptFile(std::string file)
{
    prompter_ptr = std::make_unique<Prompter>(file);

}
