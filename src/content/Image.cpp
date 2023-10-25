#include "Image.h"

size_t UPS::Image::count = 0;
std::vector<UPS::Image> UPS::Image::images;

//#define STB_IMAGE_IMPLEMENTATION
#include "../extern/stb_image.h"


UPS::Image::ImagePtr UPS::Image::Add(std::string file)
{
    auto filename = file.c_str();
    int w,h;
    // Load from file
    unsigned char* image_data = stbi_load(filename, &w, &h, NULL, 4);
    if (image_data == NULL){
        std::cerr << "[image] couldn't load image " << filename << std::endl;
        return ImagePtr();
    }

    ImagePtr rslt = NewPrimitive<Image>();
    rslt->width = w;
    rslt->height = h;

    // Create a OpenGL texture identifier
    glGenTextures(1, &rslt->texture);
    glBindTexture(GL_TEXTURE_2D, rslt->texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

// Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rslt->width, rslt->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);
    return rslt;
}

ImVec2 UPS::Image::getSize(std::string filename)
{
    int w,h;
    // Load from file
    unsigned char* image_data = stbi_load(filename.c_str(), &w, &h, NULL, 4);
    return ImVec2(w,h);
}

void UPS::Image::draw(const TimeObject &, const StateInSlide &sis)
{
    display(sis);
}

void UPS::Image::intro(const TimeObject& t, const StateInSlide &sis)
{
    auto sist = sis;
    sist.alpha = smoothstep(t.transitionParameter)*sis.alpha;
    display(sist);
}

void UPS::Image::outro(const TimeObject& t, const StateInSlide &sis)
{
    auto sist = sis;
    sist.alpha = smoothstep(1-t.transitionParameter)*sis.alpha;
    display(sist);
}

void UPS::Image::display(const StateInSlide &sis) const
{
    RGBA color_multiplier = ImColor(1.f,1.f,1.f,sis.alpha);
    auto P = sis.getAbsoluteAnchorPos();
    if (std::abs(sis.angle) > 0.001)
        ImageRotated((void*)(intptr_t)texture,P,ImVec2(width,height),sis.angle,color_multiplier);
    else {
        P.x -= width*0.5;
        P.y -= height*0.5;
        ImGui::SetCursorPos(P);
        ImGui::Image((void*)(intptr_t)texture, getSize(), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f), color_multiplier);
    }
}

void UPS::Image::ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle, const RGBA &color_mult)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    ImVec2 pos[4] =
        {
            center + ImRotate(ImVec2(-size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(-size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a)
        };
    ImVec2 uvs[4] =
        {
            ImVec2(0.0f, 0.0f),
            ImVec2(1.0f, 0.0f),
            ImVec2(1.0f, 1.0f),
            ImVec2(0.0f, 1.0f)
        };

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0], uvs[1], uvs[2], uvs[3], color_mult);
}
