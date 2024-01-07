#include "primitive.h"

std::vector<UPS::PrimitivePtr> UPS::Primitive::primitives;

UPS::TimeObject UPS::TimeObject::operator()(Primitive* p) const {
    auto tmp = *this;
    tmp.inner_time = p->getInnerTime();
    tmp.relative_frame_number = p->relativeSlideIndex(tmp.absolute_frame_number);
    return tmp;
}

void UPS::StateInSlide::setLabel(std::string l) {
    label = l;
    writeAtLabel(0.5,0.5,false);
    readFromLabel();
}

void UPS::StateInSlide::writeAtLabel(double x, double y,bool overwrite)
{
    if (label == "")
        return;
    system(("mkdir " + UPS::Options::PROJECT_ROOT + "cache 2>/dev/null").c_str());
    std::string filepath = UPS::Options::PROJECT_ROOT + "cache/" + label + ".pos";
    if (!io::file_exists(filepath) || overwrite){
        std::ofstream file(filepath);
        if (!file.is_open()){
            std::cerr << "couldn't open file" << std::endl;
            assert(0);
        }
        file << x << " " << y << std::endl;
    }
}

void UPS::StateInSlide::readFromLabel()
{
    if (label == "")
        return;
    std::ifstream file (UPS::Options::PROJECT_ROOT + "cache/" + label + ".pos");
    if (!file.is_open()){
        std::cerr << "couldn't read label file" << std::endl;
        assert(0);
    }
    file >> relative_anchor_pos.x >> relative_anchor_pos.y;
}
