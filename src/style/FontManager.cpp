#include "FontManager.h"

namespace UPS {

std::vector<ImFont*> FontManager::fonts;

FontID FontManager::addFont(std::string ttf_file, int size)
{
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    fonts.push_back(io.Fonts->AddFontFromFileTTF(ttf_file.c_str(), size));
    io.Fonts->Build();
    return fonts.size() -1 ;
}

ImFont *FontManager::getFont(FontID font)
{
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    auto tmp = fonts[font];
    if (!tmp->IsLoaded())
        io.Fonts->Build();
    return fonts[font];
}

}
