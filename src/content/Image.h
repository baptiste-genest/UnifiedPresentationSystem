#ifndef IMAGE_H
#define IMAGE_H

#include "ScreenPrimitive.h"
#include "GLFW/glfw3.h"
#include "../math/kernels.h"
#include "../math/geometry.h"

namespace UPS {

class Image : public ScreenPrimitive {
public:
    using ImagePtr = std::shared_ptr<Image>;

    Image() {}
    bool isValid() {return width != -1;}
    void display(const StateInSlide& sis) const;

    static ImagePtr Add(std::string filename,scalar scale = 1);

    static ImVec2 getSize(std::string filename);

private:
    static std::vector<Image> images;
    scalar scale;

    GLuint texture = 0;
    int width      = -1;
    int height     = -1;
    size_t assetId = 0;
    static size_t count;

    static void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle,const RGBA& color_mult);

    // Primitive interface
public:
    void draw(const TimeObject&, const StateInSlide &sis) override;
    void intro(const TimeObject& t, const StateInSlide &sis) override;
    void outro(const TimeObject& t, const StateInSlide &sis) override;
    Size getSize() const override {return Size(width,height);}
};

/*
class Gif : public Primitive {
public:
    using GifPtr = std::shared_ptr<Gif>;

    Gif() {}
    bool isValid() {return width != -1;}
    void display(const StateInSlide& sis) const;

    static GifPtr Add(std::string filename);

    static ImVec2 getSize(std::string filename);

private:
    static std::vector<Gif> images;

    GLuint texture = 0;
    int width      = -1;
    int height     = -1;
    size_t assetId = 0;
    static size_t count;


static void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle,const RGBA& color_mult);

    // Primitive interface
public:
    void draw(const TimeObject&, const StateInSlide &sis) override;
    void intro(const TimeObject& t, const StateInSlide &sis) override;
    void outro(const TimeObject& t, const StateInSlide &sis) override;
    Size getSize() const override {return Size(width,height);}
    bool isScreenSpace() override {return true;}
};
*/


}

#endif // IMAGE_H
