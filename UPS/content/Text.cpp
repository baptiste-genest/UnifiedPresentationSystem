#include "Text.h"


namespace UPS {

Text::TextPtr Text::Add(const std::string &content, FontID font)
{
    TextPtr rslt = NewPrimitive<Text>();
    rslt->content = content;
    rslt->fontID = font;
    return rslt;
}

void Text::display(const StateInSlide &sis) const
{
    if (fontID != -1){
        ImGui::PushFont(FontManager::getFont(fontID));
    }
    else{
        auto F = FontManager::getFont(Style::default_font);
        ImGui::PushFont(F);
    }

    auto size = ImGui::CalcTextSize(content.c_str());

    ImGuiStyle* style = &ImGui::GetStyle();
    auto old = style->Colors[ImGuiCol_Text];
    style->Colors[ImGuiCol_Text] = RGBA(ImVec4(0,0, 0, alpha));
    auto S = ImGui::GetWindowSize();

    ImGui::SetCursorPos(ImVec2(S.x*sis.relative_anchor_pos.x - size.x*0.5,S.y*sis.relative_anchor_pos.y - size.y*0.5));
    ImGui::Text(content.c_str());

    style->Colors[ImGuiCol_Text] = old;

    ImGui::PopFont();
}

void Text::intro(parameter t, const StateInSlide &sis)
{
    alpha = smoothstep(t);
    display(sis);
}

void Text::outro(parameter t, const StateInSlide &sis)
{
    alpha = 1-smoothstep(t);
    display(sis);

}

}
