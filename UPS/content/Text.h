#ifndef TEXT_H
#define TEXT_H

#include "primitive.h"
#include "../math/kernels.h"
#include "../style/Style.h"
#include "../style/FontManager.h"

namespace UPS {

class Text : public Primitive
{
public:
    Text() {}
    using TextPtr = std::shared_ptr<Text>;
    static TextPtr Add(const std::string &content,FontID font = -1);

private:
    scalar alpha = 1;
    std::string content;
    FontID fontID;
    FontSize fontSize;

    void display(const StateInSlide& sis) const;

    // Primitive interface
public:
    void draw(const TimeObject& t, const StateInSlide &sis) override {
        display(sis);
    }
    void intro(parameter t, const StateInSlide &sis) override;
    void outro(parameter t, const StateInSlide &sis) override;
};

}

#endif // TEXT_H
