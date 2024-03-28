#include "CameraView.h"

UPS::CameraViewPtr UPS::CameraView::Add(const vec& f,const vec& t, const vec &up,bool flyTo)
{
    return std::make_shared<CameraView>(toVec3(f),toVec3(t),toVec3(up),flyTo);
}

UPS::CameraViewPtr UPS::CameraView::Add(std::string file, bool flyTo)
{
    std::ifstream camfile(file);
    if (!camfile.is_open()){
        std::cerr << "invalid path " << file << std::endl;
        throw ;
    }
    std::string str((std::istreambuf_iterator<char>(camfile)),
                    std::istreambuf_iterator<char>());
    return std::make_shared<CameraView>(str,flyTo);
}
