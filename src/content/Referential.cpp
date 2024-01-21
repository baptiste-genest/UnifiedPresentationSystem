#include "Referential.h"


void UPS::PersistantPosition::setLabel(std::string l) {
    label = l;
    writeAtLabel(0.5,0.5,false);
    readFromLabel();
}

void UPS::PersistantPosition::writeAtLabel(double x, double y,bool overwrite) const
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

UPS::vec2 UPS::PersistantPosition::readFromLabel() const
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
