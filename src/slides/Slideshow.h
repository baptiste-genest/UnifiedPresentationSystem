#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"
#include "PrompterModule.h"
#include "screenshot.h"

namespace UPS {

class Slideshow : public PrompterModule
{
public:
    
    Slideshow() {}

    void nextFrame();

    void previousFrame();

    void forceNextFrame();

    void play();

    TimeTypeSec transitionTime = 0.5;

    void setInnerTime();


    inline TimeObject getTimeObject() const {
        TimeObject T;
        T.from_begin = TimeFrom(from_begin);
        T.from_action = TimeFrom(from_action);
        T.absolute_frame_number = current_slide;
        return T;
    }

    void handleDragAndDrop();

    void prompt();

    void handleTransition();

    void init(std::string project_name,std::string script_file ="",bool debug = false);


private:

    PrimitivePtr selected_primitive = nullptr;
    PrimitivePtr getPrimitiveUnderMouse(scalar x,scalar y) const;


    bool transition_done = false;
    bool backward = false;
    bool locked = true;

    bool debug;

    static void ImGuiWindowConfig();

    int visited_slide = -1;

    TimeStamp from_action,from_begin;
    size_t current_slide = 0;
    ImGuiWindowFlags window_flags = 0;
};


}

#endif // SLIDESHOW_H
