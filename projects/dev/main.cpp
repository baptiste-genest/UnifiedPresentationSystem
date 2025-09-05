#include "polyscope/polyscope.h"

#include "../../src/slope.h"
#include "imgui.h"
#include "spdlog/spdlog.h"

#include <cairo/cairo.h>
#include <librsvg-2.0/librsvg/rsvg.h>

using namespace slope;
slope::Slideshow show;


enum class LatexAlign { TopLeft, TopCenter, Center, BottomLeft, BottomCenter };

struct GlyphPlacement {
    std::string file;          // ex: "g0-69.png"
    ImVec2      position{0,0}; // position en *px manifest* (voir loader pour unités)
    ImVec2      anchorCrop{0,0}; // crop (left, top) en *px manifest*
    ImVec2      sizePx{0,0};   // taille PNG en pixels
    ImageData*  image = nullptr; // texture (renseignée après chargement)
};

class LatexGlyph : public ScreenPrimitive {
public:
    using Ptr = std::shared_ptr<LatexGlyph>;

    static Ptr Add(const std::string& manifestPath,
                   float userScale = 1.f,
                   int   maxGlyphsToDraw = -1);

    LatexGlyph() = default;
    ~LatexGlyph() = default;

    // Chargement du manifeste JSON et des textures
    bool loadFromManifest(const std::string& manifestPath);

    // -------- Overrides ScreenPrimitive --------
    void draw(const TimeObject&, const StateInSlide &sis) override;
    void playIntro(const TimeObject& t, const StateInSlide &sis) override { draw(t, sis); }
    void playOutro(const TimeObject& t, const StateInSlide &sis) override { draw(t, sis); }
    Size getSize() const override; // bbox * scaling

    // -------- API --------
    bool isValid() const { return valid && !placements.empty(); }
    void setUserScale(float s) { userScale = s; }
    void setMaxGlyphs(int n)   { maxGlyphsToDraw = n; }
    void setAlign(LatexAlign a){ align = a; boundsComputed = false; }
    void setTint(ImU32 c)      { tint = c; }
    void setOffsetPx(ImVec2 o) { userOffsetPx = o; }

private:
    // Helpers
    ImageData& getOrLoadTexture(const std::string& fullPngPath) const;
    void computeBounds() const; // calcule bbMin/bbMax dans l’espace *px manifest*
    void drawGlyph(const GlyphPlacement& gp, const StateInSlide& sis,
                   const ImVec2& baseScreenPos, float scaleScreen) const;
    ImVec2 alignmentOffsetPx() const; // offset d’alignement en *px manifest*

private:
    bool valid = false;

    // Infos manifeste
    std::string baseDir;   // dossier du manifest
    std::string outDir;    // dossier des PNG (relatif au manifest)
    float       ppu = 1.f; // pixels-per-unit si le manifeste est en *units*

    std::vector<GlyphPlacement> placements;

    // Cache textures par chemin absolu
    mutable std::unordered_map<std::string, ImageData> texCache;

    // Paramètres runtime
    float    userScale = 1.f;      // zoom utilisateur
    int      maxGlyphsToDraw = -1; // typewriter; -1 => tout
    LatexAlign align = LatexAlign::TopLeft;
    ImU32    tint = IM_COL32_WHITE;
    ImVec2   userOffsetPx{0,0};    // décalage manuel en px écran

    // BBox en *px manifest*
    mutable bool   boundsComputed = false;
    mutable ImVec2 bbMin{FLT_MAX, FLT_MAX};
    mutable ImVec2 bbMax{-FLT_MAX, -FLT_MAX};
};


using json = nlohmann::json;
namespace fs = std::filesystem;

static inline std::string dirOf(const std::string& path){
    size_t p = path.find_last_of("/");
    return (p==std::string::npos)?std::string("."):path.substr(0,p);
}

LatexGlyph::Ptr LatexGlyph::Add(const std::string& manifestPath,
                                float userScale,
                                int   maxGlyphsToDraw)
{
    auto p = NewPrimitive<LatexGlyph>();
    p->userScale = userScale;
    p->maxGlyphsToDraw = maxGlyphsToDraw;
    if (!p->loadFromManifest(manifestPath)) {
        spdlog::error("[LatexGlyph] Failed to load manifest {}", manifestPath);
    }
    return p;
}

bool LatexGlyph::loadFromManifest(const std::string& manifestPathIn)
{
    const std::string manifestPath = formatPath(manifestPathIn);
    baseDir = dirOf(manifestPath);

    std::ifstream ifs(manifestPath);
    if (!ifs) {
        spdlog::error("[LatexGlyph] Cannot open manifest {}", manifestPath);
        valid = false; return false;
    }
    json j; try { ifs >> j; } catch(...) { spdlog::error("[LatexGlyph] Bad JSON {}", manifestPath); valid=false; return false; }

    outDir = j.value("out_dir", std::string("glyphs"));
    ppu    = j.value("ppu", 1.0f);

    placements.clear();
    if (!j.contains("glyphs") || !j["glyphs"].is_array()) { valid=false; return false; }

    for (const auto& g : j["glyphs"]) {
        GlyphPlacement gp;
        gp.file = g.value("file", std::string(""));

        // Compatibilité: 1) notre manifeste (pos_units/anchor_units/size_px) 2) ancien (position/anchorCrop/sizePx)
        if (g.contains("pos_units")) {
            ImVec2 posUnits{ g["pos_units"][0].get<float>(), g["pos_units"][1].get<float>() };
            ImVec2 ancUnits{ g["anchor_units"][0].get<float>(), g["anchor_units"][1].get<float>() };
            gp.position   = ImVec2(posUnits.x * ppu, posUnits.y * ppu);
            gp.anchorCrop = ImVec2(ancUnits.x * ppu, ancUnits.y * ppu);
        } else if (g.contains("position")) {
            gp.position   = ImVec2(g["position"][0].get<float>(), g["position"][1].get<float>());
            gp.anchorCrop = ImVec2(g["anchorCrop"][0].get<float>(), g["anchorCrop"][1].get<float>());
        }

        if (g.contains("size_px")) {
            gp.sizePx = ImVec2(g["size_px"][0].get<float>(), g["size_px"][1].get<float>());
        } else if (g.contains("sizePx")) {
            gp.sizePx = ImVec2(g["sizePx"][0].get<float>(), g["sizePx"][1].get<float>());
        }

        placements.push_back(std::move(gp));
    }

    // Préchargement textures + pointer dans placements
    for (auto& gp : placements) {
        fs::path p = fs::path(baseDir) / outDir / gp.file;
        ImageData& tex = getOrLoadTexture(p.string());
        gp.image = &tex;
        if (tex.texture == 0 || tex.width<=0) {
            spdlog::warn("[LatexGlyph] Could not load {}", p.string());
        }
    }

    // Invalider bbox (recalcul on-demand)
    boundsComputed = false;
    valid = !placements.empty();
    return valid;
}

ImageData& LatexGlyph::getOrLoadTexture(const std::string& fullPngPath) const
{
    auto it = texCache.find(fullPngPath);
    if (it != texCache.end()) return it->second;
    ImageData data = loadImage(fullPngPath); // fournie par ta lib
    auto [ins, ok] = texCache.emplace(fullPngPath, data);
    return ins->second;
}

void LatexGlyph::computeBounds() const
{
    if (boundsComputed || placements.empty()) return;
    ImVec2 minv{FLT_MAX, FLT_MAX}, maxv{-FLT_MAX, -FLT_MAX};
    for (const auto& gp : placements) {
        ImVec2 p0 = ImVec2(gp.position.x - gp.anchorCrop.x,
                           gp.position.y - gp.anchorCrop.y);
        ImVec2 p1 = ImVec2(p0.x + gp.sizePx.x,
                           p0.y + gp.sizePx.y);
        minv.x = std::min(minv.x, std::min(p0.x, p1.x));
        minv.y = std::min(minv.y, std::min(p0.y, p1.y));
        maxv.x = std::max(maxv.x, std::max(p0.x, p1.x));
        maxv.y = std::max(maxv.y, std::max(p0.y, p1.y));
    }
    bbMin = minv; bbMax = maxv; boundsComputed = true;
}

ImVec2 LatexGlyph::alignmentOffsetPx() const
{
    // Offset à ajouter pour que le pivot corresponde au point d’alignement désiré
    const_cast<LatexGlyph*>(this)->computeBounds();
    ImVec2 ext{ std::max(0.f, bbMax.x - bbMin.x), std::max(0.f, bbMax.y - bbMin.y) };
    switch (align) {
        case LatexAlign::TopLeft:      return ImVec2(-bbMin.x, -bbMin.y);
        case LatexAlign::TopCenter:    return ImVec2(-bbMin.x - ext.x*0.5f, -bbMin.y);
        case LatexAlign::Center:       return ImVec2(-bbMin.x - ext.x*0.5f, -bbMin.y - ext.y*0.5f);
        case LatexAlign::BottomLeft:   return ImVec2(-bbMin.x, -bbMin.y - ext.y);
        case LatexAlign::BottomCenter: return ImVec2(-bbMin.x - ext.x*0.5f, -bbMin.y - ext.y);
        default:                       return ImVec2(-bbMin.x, -bbMin.y);
    }
}

ScreenPrimitive::Size LatexGlyph::getSize() const
{
    const_cast<LatexGlyph*>(this)->computeBounds();
    if (!boundsComputed) return Size(0,0);

    // Mise à l’échelle écran (baseline 1920x1080 comme Image.cpp)
    double sx = Options::ScreenResolutionWidth  / 1920.0;
    double sy = Options::ScreenResolutionHeight / 1080.0;
    bool notFullHD = (Options::ScreenResolutionWidth != 1920) || (Options::ScreenResolutionHeight != 1080);
    if (!notFullHD) { sx = sy = 1.0; }

    ImVec2 ext{ bbMax.x - bbMin.x, bbMax.y - bbMin.y };
    float k = float(std::min(sx, sy)) * userScale;
    return Size(ext.x * k, ext.y * k);
}

void LatexGlyph::drawGlyph(const GlyphPlacement& gp, const StateInSlide& sis,
                           const ImVec2& baseScreenPos, float scaleScreen) const
{
    if (!gp.image || gp.image->texture == 0) return;

    RGBA color = ImColor(1.f,1.f,1.f,sis.alpha) & tint;

    ImVec2 p0 = ImVec2(
        baseScreenPos.x + scaleScreen * (gp.position.x - gp.anchorCrop.x),
        baseScreenPos.y + scaleScreen * (gp.position.y - gp.anchorCrop.y)
    );
    ImVec2 size = ImVec2(scaleScreen * gp.sizePx.x,
                         scaleScreen * gp.sizePx.y);

    if (std::abs(sis.angle) > 0.001f) {
        ImVec2 center = ImVec2(p0.x + 0.5f*size.x, p0.y + 0.5f*size.y);
        ImageRotated((ImTextureID)(intptr_t)gp.image->texture, center, size, sis.angle, color);
    } else {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 p1 = ImVec2(p0.x + size.x, p0.y + size.y);
        dl->AddImage((ImTextureID)(intptr_t)gp.image->texture, p0, p1, ImVec2(0,0), ImVec2(1,1), color);
    }
}

void LatexGlyph::draw(const TimeObject& t, const StateInSlide &sis)
{
    if (!isValid()) return;

    // Pivot écran de la primitive (comme Image::display)
    ImVec2 base = sis.getAbsolutePosition();

    // Échelle écran basées sur 1920x1080
    double sx = Options::ScreenResolutionWidth  / 1920.0;
    double sy = Options::ScreenResolutionHeight / 1080.0;
    bool notFullHD = (Options::ScreenResolutionWidth != 1920) || (Options::ScreenResolutionHeight != 1080);
    if (!notFullHD) { sx = sy = 1.0; }

    float scaleScreen = userScale * float(std::min(sx, sy));

    // Offset d’alignement en px écran
    ImVec2 alignPx = alignmentOffsetPx();
    ImVec2 baseAligned = ImVec2(base.x + userOffsetPx.x + alignPx.x * scaleScreen,
                                base.y + userOffsetPx.y + alignPx.y * scaleScreen);

    maxGlyphsToDraw = int(t.from_begin *3) % placements.size();
    int count = (maxGlyphsToDraw >= 0) ? std::min<int>(maxGlyphsToDraw, (int)placements.size())
                                       : (int)placements.size();
    for (int i = 0; i < count; ++i) {
        drawGlyph(placements[i], sis, baseAligned, scaleScreen);
    }
}

void init () {
    LatexLoader::Init(Options::ProjectPath + "test.json");

//    auto cairo_test = CairoGraphics::Add();


//    show << cairo_test;
//    show << Formula::Add("f(x) = 8x^3");

    show << LatexGlyph::Add("/home/eulerson314/dev/UnifiedPresentationSystem/scripts/equation_glyphs.json");
}



int main(int argc,char** argv) {

    show.init("dev",argc,argv);
    init();
    polyscope::view::upDir = polyscope::UpDir::YUp;

    polyscope::state::userCallback = [](){
//        ImPlot::ShowDemoWindow();
        show.play();
    };
    polyscope::show();
    return 0;
}

/*


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
=======
*/
