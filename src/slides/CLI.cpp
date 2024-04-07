#include "CLI.h"

#include "../extern/CLI11.hpp"
#include "../content/Options.h"
#include "spdlog/spdlog.h"
#include <cassert>

int parseCLI(int argc,char** argv) {
    using namespace UPS;
    CLI::App app("Unified Presentation System");
    bool clear_cache = false;

    argv = app.ensure_utf8(argv);
    app.add_flag("--clear_cache",clear_cache,"cache will be cleared and every resources of all projects will have to be regenerated");
    app.add_flag("--ignore_cache",Options::ignore_cache,"cache will be ignored and every requested resource will be regenerated");

    std::string resolution = "1920x1080";
    app.add_option("--resolution",resolution,"screen resolution (default 1920x1080)");

    CLI11_PARSE(app,argc,argv);

    if (clear_cache){
        spdlog::info("clearing cache");
        system(("rm -rf " + Options::DataPath + "cache/*").data());
        system(("rm -rf " + Options::DataPath + "formulas/*").data());
    }

    auto pos = resolution.find('x');
    if (pos == std::string::npos){
        std::cerr << "invalid resolution format" << std::endl;
        assert(false);
    }
    Options::UPS_screen_resolution_x = std::stoi(resolution.substr(0,pos));
    Options::UPS_screen_resolution_y = std::stoi(resolution.substr(pos+1));

    return 0; //ok

}
