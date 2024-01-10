#include "Slideshow.h"
#include <GL/gl.h>


void UPS::Slideshow::nextFrame()
{
    if (current_slide == slides.size()-1)
        return;
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
        p->intro(TimeObject(p->getInnerTime(),1),s.second);
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
        p->intro(TimeObject(p->getInnerTime(),1),s.second);
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

    setInnerTime();

    bool triggerIT = false;

    TimeObject T = getTimeObject();


    for (auto& s : CS)
        CS[s.first].readFromLabel();

    if (backward || !locked) {
        locked = false;
        for (auto s : CS)
            Primitive::get(s.first)->play(T,CS[s.first]);
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
                    Primitive::get(c)->play(T,st);
                }
                if (t < transitionTime){
                    T.transitionParameter = t/transitionTime;
                    for (auto ua : UA){
                        auto p = Primitive::get(ua);
                        p->outro(T(t/transitionTime),PS[ua]);
                    }
                }
                else {
                    handleTransition();
                    for (auto ub : UB){
                        Primitive::get(ub)->intro(T(t/transitionTime-1.),CS[ub]);
                    }
                }
            }
            else {
                for (auto& s : CS){
                    Primitive::get(s.first)->play(T,CS[s.first]);
                }
                locked = false;
            }
        }
        else {
            if (t < transitionTime){
                for (auto s : CS){
                    Primitive::get(s.first)->intro(T(t/transitionTime),CS[s.first]);
                }
            }
            else {
                for (auto s : CS)
                    Primitive::get(s.first)->play(T,CS[s.first]);
                locked = false;
            }
        }
    }

    prompt();

    handleDragAndDrop();

    if (ImGui::IsKeyPressed(262) && !locked){
        nextFrame();
    }
    else if (ImGui::IsKeyPressed(263)){
        previousFrame();
    }else if (ImGui::IsKeyPressed(264)){
        forceNextFrame();
    }

    if (ImGui::IsKeyPressed(67)){
        std::string file("/tmp/cam.json");
        std::ofstream camfile(file);
        camfile << polyscope::view::getCameraJson();
        std::cout << "current camera view exported at " << file << std::endl;
    }
    if (ImGui::IsKeyPressed(80)){
        static int screenshot_count = 0;
        std::string file =  "/tmp/screenshot_" + std::to_string(screenshot_count++) + ".png";
        UPS::screenshot(file);
        std::cout << "screenshot saved at " << file << std::endl;
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

void UPS::Slideshow::handleDragAndDrop()
{
    auto io = ImGui::GetIO();
    auto S = ImGui::GetWindowSize();
    auto x = double(io.MousePos.x)/S.x;
    auto y = double(io.MousePos.y)/S.y;
    if (selected_primitive == -1 && io.MouseDown[0] > 0){
        selected_primitive = getPrimitiveUnderMouse(x,y);
    }
    else if (io.MouseDown[0] == 0. && selected_primitive != -1) {
        selected_primitive = -1;
    }
    if (io.MouseDown[0] > 0 && selected_primitive != -1) {
        auto& pis = slides[current_slide][selected_primitive];
        pis.writeAtLabel(x,y,true);
    }
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
    for (auto& s : uniquePrevious(transitions[current_slide-1]))
        Primitive::get(s)->forceDisable();
    for (auto& s : uniqueNext(transitions[current_slide-1]))
        Primitive::get(s)->forceEnable();
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
    if (debug){
        current_slide = slides.size()-1;
        for (auto& s : slides.back()){
            auto p = Primitive::get(s.first);
            p->forceEnable();
            p->handleInnerTime();
            p->intro(TimeObject(p->getInnerTime(),1),s.second);
        }
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

void UPS::Slideshow::ImGuiWindowConfig()
{
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.WantCaptureMouse = false;
    io.WantCaptureKeyboard = true;
    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x,io.DisplaySize.y));
}

void UPS::Slideshow::init(std::string project_name, std::string script_file, bool debug)
{
    from_action = Time::now();
    from_begin = Time::now();

    UPS::Options::UPSPath = TOSTRING(UPS_SOURCE);
    UPS::Options::DataPath = Options::UPSPath + "/data/";
    UPS::Options::ProjectName = project_name;
    UPS::Options::ProjectPath = UPS::Options::UPSPath+std::string("/projects/")+UPS::Options::ProjectName+std::string("/");
    UPS::Options::ProjectViewsPath = UPS::Options::ProjectPath+std::string("views/");
    std::cout << "[ UPS PATH ] " << UPS::Options::UPSPath << std::endl;
    std::cout << "[ PROJECT PATH ] " << UPS::Options::ProjectPath << std::endl;
    std::cout << "[ PROJECT CACHE PATH ] " << UPS::Options::ProjectViewsPath << std::endl;

    if (!script_file.empty())
        setScriptFile(UPS::Options::ProjectPath+script_file);
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

UPS::PrimitiveID UPS::Slideshow::getPrimitiveUnderMouse(scalar x,scalar y) const
{
    for (auto& pis : slides[current_slide]){
        auto p = pis.second.relative_anchor_pos;
        auto sp = Primitive::get(pis.first);
        if (!sp->isScreenSpace())
            continue;
        auto rs = sp->getRelativeSize();
        if (std::abs(p.x - x) < rs.x && std::abs(p.y - y) < rs.y){
            return pis.first;
        }
    }
    return -1;
}
