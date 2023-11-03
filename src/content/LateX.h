#ifndef LATEX_H
#define LATEX_H

#include "../UPS.h"
#include "Image.h"
#include "io.h"
#include <stdarg.h>
#include <string>

namespace UPS {

using TexObject = std::string;

constexpr scalar TITLE = 0.07;

void generate_latex(const std::string& filename,const TexObject& tex,bool formula,scalar height_ratio);

struct Latex {
    using LatexPtr = std::shared_ptr<Image>;
    static LatexPtr Add(const TexObject& tex,scalar height_ratio=0.04);
};

struct Formula {
    using LatexPtr = std::shared_ptr<Image>;
    static LatexPtr Add(const TexObject& tex,scalar height_ratio=0.04);
};



inline Latex::LatexPtr Title(TexObject s) {
    auto rslt = Latex::Add(s,TITLE);
    rslt->exclusive = true;
    return rslt;
}


namespace tex {

inline TexObject frac(const TexObject& A,const TexObject& B) {
    return "\\frac{" + A +"}{" + B + "}";
}

inline TexObject dot(const TexObject& A,const TexObject& B) {
    return "\\langle{" + A +"," + B + "\\rangle}";
}

inline TexObject transpose(const TexObject& A) {
    return A + "^t";
}

inline TexObject text(const TexObject& A) {
    return "\\text{" + A + "}";
}


inline TexObject center(const TexObject& A) {
    return "\\begin{center} \n" + A + "\n\\end{center}";
}

inline TexObject equation(const TexObject& A,bool number = false) {
    if (number)
        return "\\begin{equation} \n" + A + "\n\\end{equation}";
    return "\\begin{equation*} \n" + A + "\n\\end{equation*}";
}

inline TexObject pow(const TexObject& A,const TexObject& B) {
    return A + "^{" + B + "}";
}

inline TexObject AaboveB(const TexObject& A,const TexObject& B) {
    return "\\overset{" + A + "}{" + B + "}";
}

inline TexObject AbelowB(const TexObject& A,const TexObject& B) {
    return "\\underset{" + A + "}{" + B + "}";
}

template<int N = 1>
inline TexObject del(int i,bool xyz = true) {
    TexObject D;
    if (N == 1)
        D = "\\partial ";
    else
        D = "\\partial^{" + std::to_string(N) + "}";
    if (xyz) {
        if (i == 0)
            return D + "x";
        else if (i == 1)
            return D + "y";
        return D + "z";
    }
    return D + "x_{"+std::to_string(i)+"}";
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

inline TexObject cases(const std::vector<TexObject>& texs) {
    TexObject rslt = "\\begin{cases}\n";
    for (int i= 0;i<texs.size();i+=2)
        rslt += texs[i] + " ,&\\quad " + texs[i+1] + " \\\\";
    rslt += "\\end{cases}";
    return rslt;
}

template <typename... ARGS>
TexObject cases(ARGS... arguments) {
    std::vector<TexObject> data;
    data.reserve(sizeof...(arguments));
    (data.emplace_back(arguments), ...);
    return cases(data);
}


template <int col,int row>
inline TexObject Mat(const std::vector<TexObject>& texs) {
    TexObject rslt = "\\begin{pmatrix}\n";
    for (int j = 0;j<row;j++){
        for (int i= 0;i<col-1;i++)
            rslt += texs[j*col + i]+ " & " ;
        if (j < row-1 )
            rslt += texs[j*col + (col-1)] + " \\\\ ";
        else
            rslt += texs[j*col + (col-1)];
    }
    rslt += "\\end{pmatrix}";
    return rslt;
}

template <int col,int row,typename... ARGS>
TexObject Mat(ARGS... arguments) {
    std::vector<TexObject> data;
    data.reserve(sizeof...(arguments));
    (data.emplace_back(arguments), ...);
    return Mat<col,row>(data);
}

}

}

#endif // LATEX_H
