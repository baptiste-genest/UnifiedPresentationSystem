#include "LateX.h"

UPS::Latex::LatexPtr UPS::Latex::Add(const TexObject &tex,scalar height_ratio)
{
    auto H = std::hash<std::string>{}(tex+"0"+std::to_string(height_ratio));
    std::string filename = UPS_prefix + "formulas/" + std::to_string(H) + ".png";

    if (!io::file_exists(filename)){
        generate_latex(filename,tex,false,height_ratio);
    }
    return Image::Add(filename.c_str());
}

void UPS::generate_latex(const std::string &filename,
                         const TexObject &tex,
                         bool formula, 
                         scalar height_ratio)
{
    std::ofstream formula_file("/tmp/formula.tex");
    formula_file << "\\documentclass[varwidth,border=3pt]{standalone}" << std::endl;
    formula_file << "\\usepackage{standalone}" << std::endl;
    formula_file << "\\usepackage{amsmath}"<< std::endl;
    formula_file << "\\usepackage{amsfonts}"<< std::endl;
    formula_file << "\\usepackage{aligned-overset}"<< std::endl;
    formula_file << "\\usepackage{ragged2e}"<< std::endl;
    formula_file << "\\begin{document}"<< std::endl;
    if (formula)
        formula_file << "\\begin{align*}"<< std::endl;
    formula_file << tex  << std::endl;
    if (formula)
        formula_file << "\\end{align*}"<< std::endl;
    formula_file << "\\end{document}"<< std::endl;

    std::string latex_cmd = "source ~/.zshrc ;pdflatex -output-directory=/tmp /tmp/formula.tex  >>/tmp/UPS.log";
    if (std::system(latex_cmd.c_str())) {
        std::cerr << "[error while generating latex] " <<  std::endl;
        assert(false);
    }
    if (std::system("source ~/.zshrc ; pdfcrop /tmp/formula.pdf  >>/tmp/UPS.log")){
        std::cerr << "[error while croping pdf] " << std::endl;
        assert(false);
    }
    if (std::system(("source ~/.zshrc ; convert -density 800 -quality 100 /tmp/formula-crop.pdf -colorspace RGB " + filename + " >>/tmp/UPS.log").c_str())) {
        std::cerr << "[error while converting to png]" << std::endl;
        assert(false);
    }
    int h = 1080*height_ratio*Image::getSize(filename).y/99.;
    if (std::system(("source ~/.zshrc ; convert " + filename + " -resize x" + std::to_string(h) + " " + filename + " >>/tmp/UPS.log").c_str())){
        std::cerr << "[error while resizing]" << std::endl;
        assert(false);
    }
}

UPS::Formula::LatexPtr UPS::Formula::Add(const TexObject &tex, scalar height_ratio)
{
    auto H = std::hash<std::string>{}(tex+"1"+std::to_string(height_ratio));
    std::string filename = UPS_prefix + "formulas/" + std::to_string(H) + ".png";

    if (!io::file_exists(filename)){
        generate_latex(filename,tex,true,height_ratio);
    }
    return Image::Add(filename.c_str());
}
