#ifndef PROMPTERMODULE_H
#define PROMPTERMODULE_H

#include "Prompter.h"
#include "SlideManager.h"

namespace slope {

class PrompterModule : public SlideManager
{
protected:
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
public:
    void setScriptFile(std::string file) {
        prompter_ptr = std::make_unique<Prompter>(file);
    }

    void setPromptTag(promptTag tag) {
        if (prompter_ptr == nullptr){
            return;
            std::cerr << " [ Must set prompt file ]" << std::endl;
            assert(0);
        }
        if (getNumberSlides() == 0){
            std::cerr << "[ NO CURRENT SLIDE ]" << std::endl;
            assert(0);
        }
        if (!scripts_ranges.empty())
            if (scripts_ranges.back().end == -1)
                scripts_ranges.back().end = getNumberSlides()-2;
        scripts_ranges.emplace_back(getNumberSlides()-1,-1,tag);
    }
    void closePromptTag() {
        if (prompter_ptr == nullptr){
            std::cerr << " [ Must set prompt file ]" << std::endl;
            assert(0);
        }
        scripts_ranges.back().end = getNumberSlides()-1;
    }
};

inline PrompterModule& operator<<(PrompterModule& PM,promptTag tag) {
    PM.setPromptTag(tag);
    return PM;
}

struct ClosePromptTag {};
inline PrompterModule& operator<<(PrompterModule& PM,ClosePromptTag) {
    PM.closePromptTag();
    return PM;
}

}

#endif // PROMPTERMODULE_H
