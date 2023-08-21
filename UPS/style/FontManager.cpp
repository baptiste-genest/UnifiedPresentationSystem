#include "FontManager.h"

namespace UPS {

std::vector<ImFont*> FontManager::fonts;

FontID FontManager::addFont(std::string ttf_file, int size)
{
    fonts.push_back({});
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    fonts.push_back(io.Fonts->AddFontFromFileTTF(ttf_file.c_str(), size));
    return fonts.size() -1 ;
}

ImFont *FontManager::getFont(FontID font)
{
    return fonts[font];
}

}
