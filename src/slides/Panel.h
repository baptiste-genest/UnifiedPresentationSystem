#ifndef PANEL_H
#define PANEL_H

#include "../content/Anchor.h"
#include "../content/Placement.h"
#include "../content/ScreenPrimitive.h"
#include "SlideManager.h"
#include "Slide.h"

namespace slope {

class SlideManager;

class Panel {
    vec2 meanpos;
    AnchorPtr anchor;
    ScreenPrimitiveInSlide last_inserted = {nullptr,StateInSlide()};
    ScreenPrimitivePtr root = nullptr;
    bool reveal;
    Slide buffer;

    std::vector<RelativePlacement> rel_hist;

public:
    Panel(bool reveal = false,vec2 p = CENTER) : reveal(reveal) {
        anchor = AbsoluteAnchor::Add(p);
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
        auto meanpos = computeOffsetToMean(buffer);
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

inline SlideManager& operator<<(SlideManager& sm,Panel& p) {
    p.addToSlideManager(sm);
    return sm;
}

}

#endif // PANEL_H
