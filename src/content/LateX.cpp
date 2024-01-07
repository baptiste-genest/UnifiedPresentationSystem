#include "LateX.h"
#include <spdlog/spdlog.h>
#include "Options.h"

UPS::Latex::LatexPtr UPS::Latex::Add(const TexObject &tex,scalar height_ratio)
{
    auto H = std::hash<std::string>{}(tex+"0"+std::to_string(height_ratio));
    std::string filename = Options::UPS_data_prefix + "formulas/" + std::to_string(H) + ".png";

    if (!io::file_exists(filename)){
        generate_latex(filename,tex,false,height_ratio);
    }
    return Image::Add(filename.c_str());
}

void UPS::Latex::DeclareMathOperator(const TexObject &name, const TexObject &content) {
    context += "\\DeclareMathOperator*{\\" + name + "}{" + content + "}";
}

UPS::TexObject UPS::Latex::context = "";

void UPS::generate_latex(const std::string &filename,
                         const TexObject &tex,
                         bool formula, 
                         scalar height_ratio)
{
    spdlog::info("Generating latex for '{}'...", tex);
    std::ofstream formula_file("/tmp/formula.tex");
    formula_file << "\\documentclass[varwidth,border=3pt]{standalone}" << std::endl;
    formula_file << "\\usepackage{standalone}" << std::endl;
    formula_file << "\\usepackage{amsmath}"<< std::endl;
    formula_file << "\\usepackage{amsfonts}"<< std::endl;
    formula_file << "\\usepackage{aligned-overset}"<< std::endl;
    formula_file << "\\usepackage{ragged2e}"<< std::endl;

    formula_file << Latex::context << std::endl;
    formula_file << "\\begin{document}"<< std::endl;
    if (formula)
        formula_file << "\\begin{align*}"<< std::endl;
    formula_file << tex  << std::endl;
    if (formula)
        formula_file << "\\end{align*}"<< std::endl;
    formula_file << "\\end{document}"<< std::endl;

    std::string latex_cmd = Options::UPS_pathPDFLATEX+"pdflatex -output-directory=/tmp /tmp/formula.tex  >>/tmp/UPS.log";
    if (std::system(latex_cmd.c_str())) {
        std::cerr << "[error while generating latex] " <<  std::endl;
        assert(false);
    }
    if (std::system((Options::UPS_pathCONVERT+"convert -trim -density "+std::to_string(Options::UPS_density)+" -quality 100 /tmp/formula.pdf -colorspace RGB " + filename + " >>/tmp/UPS.log").c_str())) {
        std::cerr << "[error while converting to png]" << std::endl;
        assert(false);
    }
    int h = 1080*height_ratio*Image::getSize(filename).y/99.;
  spdlog::info((Options::UPS_pathCONVERT+"convert " + filename + " -resize x" + std::to_string(h) + " " + filename + " >>/tmp/UPS.log"));
    if (std::system((Options::UPS_pathCONVERT+"convert " + filename + " -resize x" + std::to_string(h) + " " + filename + " >>/tmp/UPS.log").c_str())){
        std::cerr << "[error while resizing]" << std::endl;
        assert(false);
    }
    spdlog::info("Generating latex for '{}' done.", tex);

}

UPS::Formula::LatexPtr UPS::Formula::Add(const TexObject &tex, scalar height_ratio)
{
    auto H = std::hash<std::string>{}(tex+"1"+std::to_string(height_ratio));
    std::string filename = Options::UPS_data_prefix + "formulas/" + std::to_string(H) + ".png";

    if (!io::file_exists(filename)){
        generate_latex(filename,tex,true,height_ratio);
    }
    return Image::Add(filename.c_str());
}
