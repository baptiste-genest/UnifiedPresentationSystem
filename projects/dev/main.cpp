#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"
#include "spdlog/spdlog.h"

#include <cairo/cairo.h>
#include <librsvg-2.0/librsvg/rsvg.h>

using namespace slope;
slope::Slideshow show;


// Function to draw something using Cairo (e.g., a circle that moves)
void DrawWithCairo(cairo_t* cr, float time) {
    // Clear the canvas with white
    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);  // white
    cairo_paint(cr);

    // Set drawing color (blue)
    cairo_set_source_rgb(cr, 0.0, 0.0, 1.0);

    // Animation: Move the circle based on time
    double x = 200 + 100 * sin(time); // Circle moves in a sine wave
    double y = 200 + 100 * cos(time);

    // Draw the circle
    cairo_arc(cr, x, y, 50, 0, 2 * M_PI);
    cairo_fill(cr);
}

class CairoGraphics : public ScreenPrimitive {
public:
    using CairoGraphicsPtr = std::shared_ptr<CairoGraphics>;

    ImageData tex_data;
    cairo_t* cr;
    cairo_surface_t* surface;

    ~CairoGraphics() {
        tex_data.texture = 0;
        cairo_destroy(cr);
    }

    CairoGraphics() {
        tex_data.width = 800;
        tex_data.height = 800;
        surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, tex_data.width,tex_data.height);
        cr = cairo_create(surface);


    }

    void draw(const TimeObject& t, const StateInSlide& sis) override {
        // Draw your content
        cairo_set_source_rgb(cr, 1, 1, 1);  // red
        cairo_paint(cr);

        cairo_set_source_rgb(cr, 0, 0, 1);  // red
//        cairo_set_line_width(cr, 5);

        // Animation: Move the circle based on time
        double x = 200 + 100 * std::sin(t.inner_time*5.); // Circle moves in a sine wave
        double y = 200 + 100 * std::cos(t.inner_time*5.);

        // Draw the circle
        cairo_arc(cr, x, y, 50, 0, 2 * M_PI);
        cairo_fill(cr);


        RsvgHandle *handle = rsvg_handle_new_from_file("/tmp/image.svg", nullptr);
        scalar s = 10;
        cairo_translate(cr,0.5,0.5);
        cairo_scale(cr,s,s);
        rsvg_handle_render_cairo(handle, cr);
        cairo_scale(cr,1./s,1/s);
        cairo_translate(cr,0.5,0.5);

//        cairo_surface_flush(surface);

        // Get pixel data
        unsigned char* data = cairo_image_surface_get_data(surface);
        glGenTextures(1, &tex_data.texture);
        glBindTexture(GL_TEXTURE_2D, tex_data.texture);

        // Upload Cairo pixel data (BGRA format)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_data.width, tex_data.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//        glBindTexture(GL_TEXTURE_2D, tex_data.texture);
        ImGui::Image((void*)(intptr_t)tex_data.texture, ImVec2(tex_data.width, tex_data.height));
    }

    static CairoGraphicsPtr Add() {
        CairoGraphicsPtr ptr = NewPrimitive<CairoGraphics>();
        return ptr;
    }


    // Primitive interface
protected:
    virtual void playIntro(const TimeObject &t, const StateInSlide &sis) override {}
    virtual void playOutro(const TimeObject &t, const StateInSlide &sis) override {}

    // ScreenPrimitive interface
public:
    virtual vec2 getSize() const override {
        return vec2(tex_data.width, tex_data.height);
    }
};


void init () {
    LatexLoader::Init(Options::ProjectPath + "test.json");

//    auto cairo_test = CairoGraphics::Add();


//    show << cairo_test;
    show << Formula::Add("f(x) = 3x^3")->at("test");

    auto pc = Point::Add(vec(0,0,0));
//    pc.pc.getT
    show << pc;

    show << inNextFrame << Latex::Add("test")->at("test2");
//    polyscope::pickAtScreenCoords(0.5, 0.5); // This will trigger the pick callback
}



int main(int argc,char** argv) {

    show.init("dev",argc,argv);
    init();
    polyscope::view::upDir = polyscope::UpDir::YUp;

    polyscope::state::userCallback = [](){
//        ImPlot::ShowDemoWindow();
        show.play();
//        ImGui::ShowDemoWindow();
    };
    polyscope::show();
    return 0;
}
