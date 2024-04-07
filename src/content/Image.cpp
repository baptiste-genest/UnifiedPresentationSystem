#include "Image.h"

size_t UPS::Image::count = 0;
std::vector<UPS::Image> UPS::Image::images;

//#define STB_IMAGE_IMPLEMENTATION
#include "../extern/stb_image.h"

#include <spdlog/spdlog.h>

UPS::Image::ImagePtr UPS::Image::Add(std::string file,scalar scale)
{
    ImagePtr rslt = NewPrimitive<Image>();
    rslt->data = loadImage(file);
    rslt->scale = scale;
    return rslt;
}

ImVec2 UPS::Image::getSize(std::string filename)
{
    int w,h;
    // Load from file
    unsigned char* image_data = stbi_load(filename.c_str(), &w, &h, NULL, 4);
    return ImVec2(w,h);
}

UPS::Primitive::Size UPS::Image::getScaledSize(const ImageData &data, scalar scale)
{
    double sx = scale,sy = scale;
    bool notFullHD = (Options::UPS_screen_resolution_x != 1920) ||(Options::UPS_screen_resolution_y != 1080);
    if (notFullHD){
        sx *=  Options::UPS_screen_resolution_x/1920.;
        sy *=  Options::UPS_screen_resolution_y/1080.;
    }
    return Size(sx*data.width,sy*data.height);
}

UPS::ImageData UPS::loadImage(std::string file)
{
    if (file[0] != '/'){
        //spdlog::info("relative path given, assumes project folder");
        file = UPS::Options::ProjectPath + file;
    }
    auto filename = file.c_str();
    int w,h;
    // Load from file
    unsigned char* image_data = stbi_load(filename, &w, &h, NULL, 4);
    if (image_data == NULL){
        std::cerr << "[image] couldn't load image " << filename << std::endl;
        exit(1);
    }

    ImageData data;
    data.width = w;
    data.height = h;

    // Create a OpenGL texture identifier
    glGenTextures(1, &data.texture);
    glBindTexture(GL_TEXTURE_2D, data.texture);


    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same


// Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.width, data.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);
    return data;
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

UPS::Primitive::Size UPS::Image::getSize() const {
    return getScaledSize(data,scale);
}

void UPS::Image::display(const StateInSlide &sis) const
{
    anchor->updatePos(sis.getPosition());
    DisplayImage(data,sis,scale);
}

void UPS::ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle, const RGBA &color_mult)
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

void UPS::DisplayImage(const ImageData &data, const StateInSlide &sis, scalar scale)
{
    RGBA color_multiplier = ImColor(1.f,1.f,1.f,sis.alpha);
    auto P = sis.getAbsolutePosition();

    bool notfullHD = (Options::UPS_screen_resolution_x != 1920) ||(Options::UPS_screen_resolution_y != 1080);

    if (std::abs(sis.angle) > 0.001 || std::abs(1-scale) > 1e-2 || notfullHD){
        double sx =  Options::UPS_screen_resolution_x/1920.;
        double sy =  Options::UPS_screen_resolution_y/1080.;
        ImageRotated((void*)(intptr_t)data.texture,P,ImVec2(sx*data.width*scale,sy*data.height*scale),sis.angle,color_multiplier);
    }
    else {
        P.x -= data.width*0.5;
        P.y -= data.height*0.5;
        ImGui::SetCursorPos(P);
        ImGui::Image((void*)(intptr_t)data.texture, ImVec2(data.width,data.height), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f), color_multiplier);
    }

}

UPS::Gif::GifPtr UPS::Gif::Add(std::string filename,int fps,scalar scale)
{
    auto data = loadGif(filename);
    GifPtr rslt = NewPrimitive<Gif>(data,fps,scale);
    return rslt;
}

void UPS::Gif::draw(const TimeObject &t, const StateInSlide &sis)
{
    current_img = (int)std::floor(t.inner_time*fps) % int(images.size());
    display(sis);
}

std::vector<UPS::ImageData> UPS::loadGif(std::string filename)
{
    auto H = std::to_string(std::hash<std::string>{}(filename));
    std::vector<UPS::ImageData> data;
    std::string folder = UPS::Options::DataPath + "cache/" + H;
    if (!io::folder_exists(folder) || Options::ignore_cache){
        spdlog::info("Decomposing gif " + filename);
        system(("rm -rf " + folder + " 2> /dev/null").data());
        system(("mkdir " + folder).data());
        system(("convert "+filename+" -coalesce " + folder + "/gif_%05d.png").data());
    }
    auto images = io::list_directory(folder);
    for (auto& f : images){
        data.push_back(loadImage(f));
    }
    return data;
}
