#ifndef SLIDESHOW_H
#define SLIDESHOW_H

#include "Slide.h"
#include "../content/Placement.h"
#include "Prompter.h"

namespace UPS {

class Slideshow
{
public:

    Slideshow(bool debug = false) : debug(debug) {
        from_action = Time::now();
        from_begin = Time::now();
    }

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
        last_primitive_inserted = {ptr,sis};
        slides.back().add(ptr,sis);
    }

    struct in_next_frame{};
    struct new_frame{};

    inline Slideshow& operator<<(const Slide& S) {
        addSlide(S);
        return *this;
    }


    inline Slideshow& operator<<(in_next_frame) {
        duplicateLastSlide();
        return *this;
    }

    inline Slideshow& operator<<(new_frame) {
        slides.push_back(Slide());
        return *this;
    }
    
    inline Slideshow& operator<<(const RelativePlacement& P) {
        auto pos = P.computePlacement(last_primitive_inserted);
        addToLastSlide(P.ptr,StateInSlide(pos));
        return *this;
    }


    inline Slideshow& operator<<(PrimitiveInSlide obj) {
        addToLastSlide(obj.first,obj.second);
        return *this;
    }

    inline Slideshow& operator<<(PrimitivePtr ptr) {
        addToLastSlide(ptr,{StateInSlide(CENTER)});
        return *this;
    }

    inline Slideshow& operator<<(const PrimitiveGroup& G) {
        for (const auto& ps : G)
            addToLastSlide(ps.first,ps.second);
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
        return TimeObject{TimeFrom(from_begin),TimeFrom(from_action),(int)current_slide};
    }

    void removeFromCurrentSlide(PrimitivePtr ptr) {
        slides.back().remove(ptr);
    }

    void removeFromCurrentSlide(const PrimitiveGroup& G) {
        for (const auto& ps : G)
            removeFromCurrentSlide(ps.first);
    }

    inline Slideshow& operator>>(PrimitivePtr ptr) {
        removeFromCurrentSlide(ptr);
        return *this;
    }

    inline Slideshow& operator>>(const PrimitiveGroup& G) {
        removeFromCurrentSlide(G);
        return *this;
    }


    void prompt();

    void handleTransition();

    void init(std::string script_file ="");

    void setScriptFile(std::string file);

private:

    struct prompt_range {
        int begin,end;
        promptTag tag;
        bool inRange(int c) const {
            if (end == -1)
                return begin <= c;
            return (begin <= c) && (c <= end);
        }
    };
    std::vector<prompt_range> scripts_ranges;
    std::unique_ptr<Prompter> prompter_ptr;

    PrimitiveInSlide last_primitive_inserted;

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
constexpr Slideshow::new_frame newFrame;

}

#endif // SLIDESHOW_H
