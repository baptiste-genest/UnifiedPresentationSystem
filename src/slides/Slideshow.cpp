#include "Slideshow.h"
//#include <GL/gl.h>


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
        s->forceDisable();
    for (auto& s : uniquePrevious(transitions[current_slide-1]))
        s->forceEnable();
    current_slide--;
    for (auto& s : slides[current_slide]){
        auto p = s.first;
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
        s->forceDisable();
    for (auto& s : uniqueNext(transitions[current_slide]))
        s->forceEnable();
    current_slide++;
    for (auto& s : slides[current_slide]){
        auto p = s.first;
        p->intro(TimeObject(p->getInnerTime(),1),s.second);
    }
    from_action = Time::now();
    backward = false;
    locked = false;
}

void UPS::Slideshow::play() {
    ImGuiWindowConfig();
    ImGui::Begin("Unified Presentation System",NULL,window_flags);

    if (!initialized)
        initialize_slides();

    auto t = TimeFrom(from_action);

    auto& CS = slides[current_slide];

    setInnerTime();

    bool triggerIT = false;

    TimeObject T = getTimeObject();


    if (backward || !locked) {
        locked = false;
        for (auto& s : CS)
            s.first->play(T,CS[s.first]);
    }
    else {
        if (current_slide > 0){
            auto& PS = slides[current_slide-1];
            auto&& [common,UA,UB] = transitions[current_slide-1];
            if (t < 2*transitionTime){
                for (auto& c : common) {
                    auto st = transition(0.5*t/transitionTime,
                                         PS[c],
                                         CS[c]);
                    c->play(T,st);
                }
                if (t < transitionTime){
                    T.transitionParameter = t/transitionTime;
                    for (auto& ua : UA){
                        auto p = ua;
                        p->outro(T(t/transitionTime),PS[ua]);
                    }
                }
                else {
                    handleTransition();
                    for (auto& ub : UB){
                        ub->intro(T(t/transitionTime-1.),CS[ub]);
                    }
                }
            }
            else {
                for (auto& s : CS){
                    s.first->play(T,CS[s.first]);
                }
                locked = false;
            }
        }
        else {
            if (t < transitionTime){
                for (auto& s : CS){
                    s.first->intro(T(t/transitionTime),CS[s.first]);
                }
            }
            else {
                for (auto& s : CS)
                    s.first->play(T,CS[s.first]);
                locked = false;
            }
        }
    }

    prompt();

    handleDragAndDrop();

    if (ImGui::IsKeyPressed(262) && !locked){ // RIGHT ARROW
        nextFrame();
    }
    else if (ImGui::IsKeyPressed(263)){// LEFT ARROW
        previousFrame();
    }else if (ImGui::IsKeyPressed(264)){// DOWN ARROW
        forceNextFrame();
    }
    if (ImGui::IsKeyDown(258))//TABS
        slideMenu();

    if (ImGui::IsKeyPressed(67)){// C
        std::string file("/tmp/cam.json");
        std::ofstream camfile(file);
        camfile << polyscope::view::getCameraJson();
        std::cout << "current camera view exported at " << file << std::endl;
    }
    if (ImGui::IsKeyPressed(80)){ // P
        static int screenshot_count = 0;
        constexpr int nb_zeros = 6;
        auto n = std::to_string(screenshot_count++);
        std::string file =  "/tmp/screenshot_" + std::string(nb_zeros-n.size(),'0') + n + ".png";
        UPS::screenshot(file);
        std::cout << "screenshot saved at " << file << std::endl;
    }

    if (display_slide_number)
        displaySlideNumber();

    ImGui::End();
}

void UPS::Slideshow::setInnerTime()
{
    if (visited_slide == current_slide)
        return;
    for (auto& p : appearing_primitives[current_slide])
        p->handleInnerTime();
    visited_slide = current_slide;
}

void UPS::Slideshow::handleDragAndDrop()
{
    auto io = ImGui::GetIO();
    if (!ImGui::IsKeyPressed(341) || io.MouseReleased[0] > 0){//CTRL {
        selected_primitive = nullptr;
        io.WantCaptureMouse = false;
        return;
    }

    io.WantCaptureMouse = true;

    auto S = ImGui::GetWindowSize();
    auto x = double(io.MousePos.x)/S.x;
    auto y = double(io.MousePos.y)/S.y;
    if (selected_primitive == nullptr && io.MouseDown[0] > 0){
        selected_primitive = getPrimitiveUnderMouse(x,y);
    }
    else if (io.MouseReleased[0] > 0. && selected_primitive != nullptr) {
        selected_primitive = nullptr;
    }
    if (io.MouseDown[0] > 0 && selected_primitive != nullptr) {
        auto& pis = slides[current_slide][selected_primitive];
        pis.anchor->writeAtLabel(x,y,true);
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
        s->forceDisable();
    for (auto& s : uniqueNext(transitions[current_slide-1]))
        s->forceEnable();
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

    //UPS::Options::UPSPath = TOSTRING(UPS_SOURCE);
    UPS::Options::UPSPath = Options::UPSPath;
    UPS::Options::DataPath = Options::UPSPath + "/data/";
    UPS::Options::ProjectName = project_name;
    UPS::Options::ProjectPath = UPS::Options::UPSPath+std::string("/projects/")+UPS::Options::ProjectName+std::string("/");
    UPS::Options::ProjectViewsPath = UPS::Options::ProjectPath+std::string("views/");

    std::cout << "			[ UPS PROJECT : " << UPS::Options::ProjectName << " ]" << std::endl;

    std::cout << "[ UPS PATH ] " << UPS::Options::UPSPath << std::endl;
    std::cout << "[ PROJECT PATH ] " << UPS::Options::ProjectPath << std::endl;
    std::cout << "[ PROJECT CACHE PATH ] " << UPS::Options::ProjectViewsPath << std::endl;

    std::cout << "[ keys description ] " << std::endl;
    std::cout << "  - right arrow : next slide" << std::endl;
    std::cout << "  - left arrow : previous slide" << std::endl;
    std::cout << "  - down arrow : skip to next slide without transition" << std::endl;
    std::cout << "  - tab : slide menu" << std::endl;
    std::cout << "  - c : export current camera view" << std::endl;
    std::cout << "  - p : take a screenshot" << std::endl;
    std::cout << "  - ctrl + left click : drag labeled screen primitives" << std::endl;

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

    polyscope::options::ssaaFactor = 2;

    window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoResize;
    window_flags |= ImGuiWindowFlags_NoBackground;
    window_flags |= ImGuiWindowFlags_NoScrollbar;

}

void UPS::Slideshow::slideMenu()
{
    ImGui::Begin("Slides");
    std::set<std::string> done;
    for (int i = 0;i<slides.size();i++){
        auto title = getSlideTitle(i);
        if (done.contains(title))
            continue;
        done.insert(title);
        if (ImGui::Button(title.c_str()))
            goToSlide(i);
    }
    ImGui::End();
}

std::string UPS::Slideshow::getSlideTitle(int i)
{
    auto title = slides[i].getTitle();
    if (title == "")
        title = std::to_string(i);
    return title;
}

void UPS::Slideshow::goToSlide(int slide_nb)
{
    if (slide_nb == current_slide)
        return;
    for (auto& p : slides[current_slide])
        p.first->forceDisable();
    current_slide = slide_nb;
    for (auto& p : slides[current_slide])
        p.first->forceEnable();
    from_action = Time::now();
    from_action = Time::now();
}

void UPS::Slideshow::initialize_slides()
{
    precomputeTransitions();
    loadSlides();
    from_begin = Time::now();
}

void UPS::Slideshow::loadSlides()
{
    slide_numbers.resize(slides.size());
    std::set<std::string> done;

    for (int i = 0;i<slides.size();i++){
        done.insert(getSlideTitle(i));
        slide_numbers[i] = done.size()-1;
    }
    nb_distinct_slides = done.size();

    std::cout << "nb distinct slides " << nb_distinct_slides << std::endl;

    slide_number_display.resize(nb_distinct_slides);
    for (int i = 0;i<nb_distinct_slides;i++){
        auto display = std::to_string(i+1) + "/" + std::to_string(nb_distinct_slides);
        slide_number_display[i] = PlaceBottomRight(Text::Add(display),0.01);
    }
}


UPS::PrimitivePtr UPS::Slideshow::getPrimitiveUnderMouse(scalar x,scalar y) const
{
    auto io = ImGui::GetIO();
    auto S = ImGui::GetWindowSize();
    for (auto& pis : slides[current_slide].getScreenPrimitives()){
        if (!pis.second.anchor->isPersistant())
            continue;
        auto p = pis.second.getPosition();
        if (std::abs(p(0) - x) < pis.first->getSize()(0)/2/S.x && std::abs(p(1) - y) < pis.first->getSize()(1)/2/S.y)
            return pis.first;
    }
    return nullptr;
}

void UPS::Slideshow::displaySlideNumber()
{
    const auto& DSN = slide_number_display[slide_numbers[current_slide]];
    DSN.first->play(TimeObject(),DSN.second);
}
