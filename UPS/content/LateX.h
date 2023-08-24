#ifndef LATEX_H
#define LATEX_H

#include "../UPS.h"
#include "Image.h"
#include "io.h"

namespace UPS {

using TexObject = std::string;

struct Latex : public Image {
    using LatexPtr = std::shared_ptr<Image>;
    static LatexPtr Add(const TexObject& tex,bool formula = false);
};


namespace tex {

inline TexObject frac(const TexObject& A,const TexObject& B) {
    return "\\frac{" + A +"}{" + B + "}";
}

inline TexObject center(const TexObject& A) {
    return "\\begin{center} \n" + A + "\n\\end{center}";
}

inline TexObject equation(const TexObject& A,bool number = false) {
    if (number)
        return "\\begin{equation} \n" + A + "\n\\end{equation}";
    return "\\begin{equation*} \n" + A + "\n\\end{equation*}";
}

}

}

#endif // LATEX_H
