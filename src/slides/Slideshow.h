#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"
#include "../content/Placement.h"
#include "../content/PrimitiveGroup.h"
#include "Prompter.h"
#include "screenshot.h"
#include "../math/utils.h"

namespace UPS {

class Slideshow
{
public:
    
    Slideshow() {}

    void nextFrame();

    void previousFrame();

    void forceNextFrame();

    void play();

    void addSlide(const Slide& s) {
        transitions_computed = false;
        slides.push_back(s);
    }

    template<typename... T>
    void addSlides(const T& ... x) {
        (addSlide(x), ...);
    }

    TimeTypeSec transitionTime = 0.5;

    void setInnerTime();

    inline Slide& getCurrentSlide(){return slides.back();}
    inline Slide& getSlide(index i){return slides[i];}

    void duplicateLastSlide(){slides.push_back(slides.back());}

    void addToLastSlide(PrimitivePtr ptr,const StateInSlide& sis) {
        transitions_computed = false;
        if (slides.empty())
            slides.push_back(Slide());
        if (ptr->isScreenSpace())
            last_screen_primitive_inserted = std::static_pointer_cast<ScreenPrimitive>(ptr);
        slides.back().add(ptr,sis);
    }

    struct in_next_frame{};
    struct new_frame{
        bool same_title = false;
    };

    inline Slideshow& operator<<(const Slide& S) {
        addSlide(S);
        return *this;
    }


    inline Slideshow& operator<<(in_next_frame) {
        duplicateLastSlide();
        return *this;
    }

    inline Slideshow& operator<<(new_frame nf) {
        slides.push_back(Slide());
        if (nf.same_title){
            int i = slides.size()-2;
            auto ex = slides[i].exclusive_prim;
            if (ex != nullptr){
                addToLastSlide(ex,slides[i][ex]);
            }
        }
        return *this;
    }

    inline Slideshow& operator<<(const Replace& R) {
        PrimitiveInSlide old;
        if (!R.ptr_other)
            old = {last_screen_primitive_inserted,slides.back()[last_screen_primitive_inserted]};
        else
            old = {R.ptr_other,slides.back()[R.ptr_other]};
        auto pos = old.second;
        removeFromCurrentSlide(old.first);
        addToLastSlide(R.ptr,pos);
        return *this;
    }

    inline Slideshow& operator<<(const RelativePlacement& P) {
        StateInSlide sis;
        if (!P.ptr_other)
            sis.p = P.computePlacement(last_screen_primitive_inserted);
        else
            sis.p = P.computePlacement(P.ptr_other);
        addToLastSlide(P.ptr,sis);
        return *this;
    }


    inline Slideshow& operator<<(PrimitiveInSlide obj) {
        addToLastSlide(obj.first,obj.second);
        return *this;
    }

    inline Slideshow& operator<<(PrimitivePtr ptr) {
        addToLastSlide(ptr,StateInSlide(vec2(0.5,0.5)));
        return *this;
    }

    inline Slideshow& operator<<(const StateInSlide& sis) {
        slides.back()[last_screen_primitive_inserted] = sis;
        return *this;
    }

    inline Slideshow& operator<<(const PrimitiveGroup& G) {
        //auto mean = G.getRelativeAnchorPos();
        for (auto [ptr,sis] : G.buffer.getScreenPrimitives()){
        //    sis.relative_anchor_pos.x += G.relative_anchor.x - mean.x();
        //    sis.relative_anchor_pos.y += G.relative_anchor.y - mean.y();
            addToLastSlide(ptr,sis);
        }
        return *this;
    }

    void removeFromCurrentSlide(PrimitivePtr ptr) {
        slides.back().remove(ptr);
    }

    void removeFromCurrentSlide(const PrimitiveGroup& G) {
        for (const auto& p : G.buffer)
            removeFromCurrentSlide(p.first);
    }

    inline Slideshow& operator>>(PrimitivePtr ptr) {
        removeFromCurrentSlide(ptr);
        return *this;
    }

    inline Slideshow& operator>>(const PrimitiveGroup& G) {
        removeFromCurrentSlide(G);
        return *this;
    }

    inline Slideshow& operator<<(promptTag tag) {
        if (prompter_ptr == nullptr){
            return *this;
            std::cerr << " [ Must set prompt file ]" << std::endl;
            assert(0);
        }
        if (slides.empty()){
            std::cerr << "[ NO CURRENT SLIDE ]" << std::endl;
            assert(0);
        }
        if (!scripts_ranges.empty())
            if (scripts_ranges.back().end == -1)
                scripts_ranges.back().end = slides.size()-2;
        scripts_ranges.emplace_back(slides.size()-1,-1,tag);
        return *this;
    }

    struct ClosePromptTag {};
    inline Slideshow& operator<<(ClosePromptTag) {
        if (prompter_ptr == nullptr){
            std::cerr << " [ Must set prompt file ]" << std::endl;
            assert(0);
        }
        scripts_ranges.back().end = slides.size()-1;
        return *this;
    }


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

    void setScriptFile(std::string file);

private:

    PrimitivePtr selected_primitive = nullptr;
    PrimitivePtr getPrimitiveUnderMouse(scalar x,scalar y) const;

    struct prompt_range {
        int begin,end;
        promptTag tag;
        prompt_range(int b, int e,promptTag t) : begin(b),end(e),tag(t) {}
        bool inRange(int c) const {
            if (end == -1)
                return begin <= c;
            return (begin <= c) && (c <= end);
        }
    };
    std::vector<prompt_range> scripts_ranges;
    std::unique_ptr<Prompter> prompter_ptr;

    ScreenPrimitivePtr last_screen_primitive_inserted;

    bool transition_done = false;
    bool backward = false;
    bool locked = true;

    bool debug;

    static StateInSlide transition(parameter t,const StateInSlide& sa,const StateInSlide& sb);

    bool transitions_computed = true;

    using TransitionSets = std::tuple<Primitives,Primitives,Primitives>;
    Primitives& common(TransitionSets& S) {return std::get<0>(S);}
    Primitives& uniquePrevious(TransitionSets& S) {return std::get<1>(S);}
    Primitives& uniqueNext(TransitionSets& S) {return std::get<2>(S);}
    TransitionSets computeTransitionsBetween(const Slide& A,const Slide& B);
    void precomputeTransitions();

    static void ImGuiWindowConfig();

    int visited_slide = -1;

    TimeStamp from_action,from_begin;
    std::vector<Primitives> appearing_primitives;
    std::vector<Slide> slides;
    std::vector<TransitionSets> transitions;
    size_t current_slide = 0;
    ImGuiWindowFlags window_flags = 0;
};

constexpr Slideshow::in_next_frame inNextFrame;
constexpr Slideshow::new_frame newFrame{false};
constexpr Slideshow::new_frame newFrameSameTitle{true};

}

#endif // SLIDESHOW_H
