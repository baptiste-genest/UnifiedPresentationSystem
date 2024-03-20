#ifndef LATEX_H
#define LATEX_H

#include "../UPS.h"
#include "Image.h"
#include "io.h"
#include <stdarg.h>
#include <string>
#include "Options.h"

namespace UPS {

using TexObject = std::string;

void generate_latex(const std::string& filename,const TexObject& tex,bool formula,scalar height_ratio);
struct Latex;
using LatexPtr = std::shared_ptr<Latex>;

struct Latex : public TextualPrimitive {

    static LatexPtr Add(const TexObject& tex,scalar height_ratio = Options::UPS_default_height_ratio);

    static TexObject context;
    static void Define(const TexObject& tex) {context += tex;}
    static void AddToPrefix(const TexObject& tex) {context += tex;}
    static void DeclareMathOperator(const TexObject& name,const TexObject& content);
    static void NewCommand(const TexObject& name,const TexObject& content) {context += "\\newcommand{\\"+name+"}{"+content+"}";}

    static void UsePackage(std::string pkg,std::string options = "") {
        if (options != "")
            context += "\\usepackage["+options+"]{"+pkg+"}\n";
        else
            context += "\\usepackage{"+pkg+"}\n";
    }

    ImageData data;

    // Primitive interface
public:
    Latex() {}
    ~Latex() {}
    void display(const StateInSlide& sis) const{
        anchor->updatePos(sis.getPosition());
        DisplayImage(data,sis);
    }
    virtual void draw(const TimeObject &time, const StateInSlide &sis) override {
        display(sis);
    }
    virtual void intro(const TimeObject &t, const StateInSlide &sis) override {
        auto sist = sis;
        sist.alpha = smoothstep(t.transitionParameter)*sis.alpha;
        display(sist);
    }
    virtual void outro(const TimeObject &t, const StateInSlide &sis) override {
        auto sist = sis;
        sist.alpha = smoothstep(1-t.transitionParameter)*sis.alpha;
        display(sist);
    }

    // ScreenPrimitive interface
public:
    virtual vec2 getSize() const override {
        bool notfullHD = (Options::UPS_screen_resolution_x != 1920) ||(Options::UPS_screen_resolution_y != 1080);
        if (notfullHD){
            double sx =  Options::UPS_screen_resolution_x/1920.;
            double sy =  Options::UPS_screen_resolution_y/1080.;
            return vec2(sx*data.width,sy*data.height);
        }
        return vec2(data.width,data.height);
    }
};

struct Formula : public Latex {
    static LatexPtr Add(const TexObject& tex,scalar height_ratio = Options::UPS_default_height_ratio);
};




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

inline TexObject align(const std::vector<TexObject>& texs) {
    TexObject rslt = "\\begin{align*}\n";
    for (int i= 0;i<texs.size()-1;i++)
        rslt += texs[i] + "\\\\ \n";
    rslt += texs.back();
    rslt += "\n \\end{align*}";
    return rslt;
}

template <typename... ARGS>
TexObject align(ARGS... arguments) {
    std::vector<TexObject> data;
    data.reserve(sizeof...(arguments));
    (data.emplace_back(arguments), ...);
    return align(data);
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

inline TexObject enumerate(const std::vector<TexObject>& texs) {
    TexObject rslt = "\\begin{enumerate}\n";
    for (int i= 0;i<texs.size();i++)
        rslt +=  "\\item " + texs[i] + '\n';
    rslt += "\\end{enumerate}";
    return rslt;
}

template <typename... ARGS>
TexObject enumerate(ARGS... arguments) {
    std::vector<TexObject> data;
    data.reserve(sizeof...(arguments));
    (data.emplace_back(arguments), ...);
    return enumerate(data);
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

inline LatexPtr Title(TexObject s,bool center = true) {
    auto old = s;
    if (center)
        s = tex::center(s);
    auto rslt = Latex::Add(s,Options::UPS_TITLE);
    rslt->exclusive = true;
    rslt->content = old;
    return rslt;
}


}

#endif // LATEX_H
