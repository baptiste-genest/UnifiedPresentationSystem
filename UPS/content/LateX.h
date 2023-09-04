#ifndef LATEX_H
#define LATEX_H

#include "../UPS.h"
#include "Image.h"
#include "io.h"
#include <stdarg.h>

namespace UPS {

using TexObject = std::string;

constexpr scalar TITLE = 0.07;

struct Latex : public Image {
    using LatexPtr = std::shared_ptr<Image>;
    static LatexPtr Add(const TexObject& tex,bool formula = false,scalar height_ratio=0.04);
};

inline Latex::LatexPtr Title(TexObject s) {
    return Latex::Add(s,false,0.07);
}


namespace tex {

inline TexObject frac(const TexObject& A,const TexObject& B) {
    return "\\frac{" + A +"}{" + B + "}";
}

inline TexObject transpose(const TexObject& A) {
    return A + "^t";
}

inline TexObject center(const TexObject& A) {
    return "\\begin{center} \n" + A + "\n\\end{center}";
}

inline TexObject equation(const TexObject& A,bool number = false) {
    if (number)
        return "\\begin{equation} \n" + A + "\n\\end{equation}";
    return "\\begin{equation*} \n" + A + "\n\\end{equation*}";
}

inline TexObject Vec(const std::vector<TexObject>& texs) {
    TexObject rslt = "\\begin{pmatrix}\n";
    for (int i= 0;i<texs.size()-1;i++)
        rslt += texs[i] + "\\\\";
    rslt += texs.back();
    rslt += "\\end{pmatrix}";
    return rslt;
}

template <typename... ARGS>
TexObject Vec(ARGS... arguments) {
    std::vector<TexObject> data;
    data.reserve(sizeof...(arguments));
    (data.emplace_back(arguments), ...);
    return Vec(data);
}


/*
inline TexObject Vec(const char* format, ...) {
    va_list args;
    va_start(args, format);

    TexObject rslt = "\\begin{pmatrix}\n";
    const char* arg = format;
    while (arg != nullptr) {
        rslt += arg;
        arg = va_arg(args, const char*);
        if (arg != nullptr)
            rslt += "\\\\ ";
    }
    rslt += "\\end{pmatrix}";

    va_end(args);
    return rslt;
}
*/

}

}

#endif // LATEX_H
