#ifndef FONTMANAGER_H
#define FONTMANAGER_H

#include "../UPS.h"

namespace UPS {


class FontManager
{
public:
    static FontID addFont(std::string ttf_file,int size);
    static ImFont* getFont(FontID font);

private:
    static std::vector<ImFont*> fonts;
    static size_t count;

};

}

#endif // FONTMANAGER_H
