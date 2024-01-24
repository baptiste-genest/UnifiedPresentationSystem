#ifndef ANCHOR_H
#define ANCHOR_H

#include "Options.h"
#include "io.h"

namespace UPS {

class Anchor;
using AnchorPtr = std::shared_ptr<Anchor>;

extern AnchorPtr GlobalAnchor;

class Anchor
{
    vec2 pos;
    std::string label = "";

    vec2 readFromLabel() const
    {
        std::ifstream file (UPS::Options::ProjectViewsPath + label + ".pos");
        if (!file.is_open()){
            std::cerr << "couldn't read label file" << std::endl;
            exit(1);
        }
        vec2 rslt;
        file >> rslt(0) >> rslt(1);
        return rslt;
    }
public:
    Anchor(const vec2& p) {pos = p;}
    Anchor(std::string l) : label(l) {
        writeAtLabel(0.5,0.5,false);
    }
    static AnchorPtr Add(const vec2& p) {
        return std::make_shared<Anchor>(p);
    }
    static AnchorPtr Add(std::string l) {
        return std::make_shared<Anchor>(l);
    }
    void writeAtLabel(double x, double y,bool overwrite) const
    {
        if (label == "")
            return;
        system(("mkdir " + UPS::Options::ProjectViewsPath + " 2>/dev/null").c_str());
        std::string filepath = UPS::Options::ProjectViewsPath + label + ".pos";
        if (!io::file_exists(filepath) || overwrite){
            std::ofstream file(filepath);
            if (!file.is_open()){
                std::cerr << "couldn't open file" << std::endl;
                exit(1);
            }
            file << x << " " << y << std::endl;
        }
    }

    bool isPersistant() const {
        return label != "";
    }


    void updatePos(const vec2& p) {
        if (label != "")
            std::cerr << "[WARNING] Anchor : changing position of persistant Anchor" << std::endl;
        pos = p;
    }

    vec2 getPos() const {
        if (label != "")
            return GlobalAnchor->pos + readFromLabel();
        return pos;
    }
};


}

#endif // ANCHOR_H
