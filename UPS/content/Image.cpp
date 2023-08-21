#include "Image.h"

size_t UPS::Image::count = 0;
std::vector<UPS::Image> UPS::Image::images;

//#define STB_IMAGE_IMPLEMENTATION
#include "../../extern/stb_image.h"


UPS::Image::ImagePtr UPS::Image::LoadImage(const char *filename)
{
    ImagePtr rslt = std::make_shared<Image>();
    // Load from file
    unsigned char* image_data = stbi_load(filename, &rslt->width, &rslt->height, NULL, 4);
    if (image_data == NULL){
        std::cerr << "[image] couldn't load image " << filename << std::endl;
        return ImagePtr();
    }

    Primitive::addPrimitive(rslt);

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

void UPS::Image::draw(TimeTypeSec, const StateInSlide &sis)
{
    display(sis);
}

void UPS::Image::intro(parameter t, const StateInSlide &sis)
{
    auto sist = sis;
    sist.alpha = smoothstep(t)*sis.alpha;
    display(sist);
}

void UPS::Image::outro(parameter t, const StateInSlide &sis)
{
    auto sist = sis;
    sist.alpha = smoothstep(1-t)*sis.alpha;
    display(sist);
}

void UPS::Image::display(const StateInSlide &sis) const
{
    auto P = sis.getAbsoluteAnchorPos();
    P.x -= width*0.5;
    P.y -= height*0.5;
    ImGui::SetCursorPos(P);
    RGBA color_multiplier(1.,1.,1.,sis.alpha);
    ImGui::Image((void*)(intptr_t)texture, getSize(), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f), color_multiplier);
}

