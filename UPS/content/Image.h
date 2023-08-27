#ifndef IMAGE_H
#define IMAGE_H

#include "primitive.h"
#include "GLFW/glfw3.h"
#include "../math/kernels.h"
#include "../math/geometry.h"

namespace UPS {

class Image : public Primitive {
public:
    using ImagePtr = std::shared_ptr<Image>;

    Image() {}
    bool isValid() {return width != -1;}
    void display(const StateInSlide& sis) const;

    static ImagePtr Add(const char* filename);

    Vec2 getSize() const {
        return Vec2(width,height);
    }

private:
    static std::vector<Image> images;

    GLuint texture = 0;
    int width      = -1;
    int height     = -1;
    size_t assetId = 0;
    static size_t count;


static void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle,const RGBA& color_mult);

    // Primitive interface
public:
    void draw(const TimeObject&, const StateInSlide &sis) override;
    void intro(parameter t, const StateInSlide &sis) override;
    void outro(parameter t, const StateInSlide &sis) override;
};

}

#endif // IMAGE_H
