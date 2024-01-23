#ifndef PANEL_H
#define PANEL_H

#include "../content/Anchor.h"
#include "../content/Placement.h"
#include "../content/ScreenPrimitive.h"
#include "SlideManager.h"
#include "Slide.h"

namespace UPS {

class SlideManager;

class Panel {
    vec2 meanpos,bbox;
    AnchorPtr anchor;
    ScreenPrimitiveInSlide last_inserted = {nullptr,StateInSlide()};
    ScreenPrimitivePtr root = nullptr;
    bool reveal;
    Slide buffer;

    vec2 bbox_x = vec2(100,0),bbox_y= vec2(100,0);

    std::vector<RelativePlacement> rel_hist;

    void computeGeometry() {
        double x_min = 100,y_min = 100,x_max = 0,y_max = 0;
        for (auto&& [ptr,sis] : buffer) {
            auto pos = sis.getPosition();
            auto size = std::dynamic_pointer_cast<ScreenPrimitive>(ptr)->getRelativeSize();
            x_min = std::min(x_min,pos(0)-size(0)*0.5f);
            y_min = std::min(y_min,pos(1)-size(1)*0.5f);
            x_max = std::max(x_max,pos(0)+size(0)*0.5f);
            y_max = std::max(y_max,pos(1)+size(1)*0.5f);
        }
        bbox = vec2(x_max-x_min,y_max-y_min);
        meanpos = vec2(x_min,y_min) + bbox*0.5f;
    }

public:
    Panel(bool reveal = false,vec2 p = CENTER) : reveal(reveal) {
        anchor = Anchor::Add(p);
    }

    AnchorPtr getAnchor() const {return anchor;}

    Panel& operator<<(ScreenPrimitivePtr ptr) {
        if (root != nullptr){
            std::cerr << "[ PANEL ERROR ] only root can be without position" << std::endl;
            exit(1);
        }
        last_inserted = {ptr,StateInSlide(vec2(0,0))};
        buffer.add(last_inserted);
        root = ptr;
        return *this;
    }

    Panel& operator<<(const RelativePlacement& P) {
        if (root == nullptr)
            std::cout << "[ PANEL ERROR ] no root" << std::endl;
        StateInSlide sis;
        if (!P.ptr_other)
            sis = P.computePlacement(last_inserted);
        else
            sis = P.computePlacement({P.ptr_other,buffer[P.ptr_other]});
        buffer[P.ptr] = sis;
        return *this;
    }

    void addToSlideManager(SlideManager& sm){
        computeGeometry();
        PrimitiveInSlide ris; ris.first = root;
        ris.second.anchor = anchor;
        ris.second.setOffset(-meanpos);
        sm << ris;
        for (auto&& [ptr,sis] : buffer) {
            if (ptr == root) continue;
            if (reveal)
                sm << inNextFrame;
            sm << PrimitiveInSlide(ptr,sis);
        }

    }
};

SlideManager& operator<<(SlideManager& sm,Panel& p) {
    p.addToSlideManager(sm);
    return sm;
}

}

#endif // PANEL_H
