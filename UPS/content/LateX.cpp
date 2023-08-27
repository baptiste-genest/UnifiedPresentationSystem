#include "LateX.h"

UPS::Latex::LatexPtr UPS::Latex::Add(const TexObject &tex,bool formula)
{
    auto H = std::hash<std::string>{}(tex);
    std::string filename = UPS_prefix + "formulas/" + std::to_string(H) + ".png";

    if (!io::file_exists(filename)){
        std::ofstream formula_file("/tmp/formula.tex");
        formula_file << "\\documentclass[varwidth]{standalone}" << std::endl;
        formula_file << "\\usepackage{standalone}" << std::endl;
        formula_file << "\\usepackage{amsmath}"<< std::endl;
        formula_file << "\\usepackage{amsfonts}"<< std::endl;
        formula_file << "\\usepackage{ragged2e}"<< std::endl;
        formula_file << "\\begin{document}"<< std::endl;
        if (formula)
            formula_file << "\\begin{align*}"<< std::endl;
        formula_file << tex  << std::endl;
        if (formula)
            formula_file << "\\end{align*}"<< std::endl;
        formula_file << "\\end{document}"<< std::endl;

        auto latex_cmd = "pdflatex /tmp/formula.tex >>/tmp/UPS.log";
        if (std::system(latex_cmd)) {
            std::cerr << "[error while generating latex] " << latex_cmd << std::endl;
            assert(false);
            return LatexPtr();
        }
        if (std::system("pdfcrop formula.pdf >>/tmp/UPS.log")){
            std::cerr << "[error while croping pdf] " << std::endl;
            assert(false);
            return LatexPtr();
        }
        if (std::system(("convert -density 500 -quality 100 formula-crop.pdf -colorspace RGB " + filename + " >>/tmp/UPS.log").c_str())) {
            std::cerr << "[error while converting to png]" << std::endl;
            assert(false);
            return LatexPtr();
        }
    }
    return Image::Add(filename.c_str());
}
