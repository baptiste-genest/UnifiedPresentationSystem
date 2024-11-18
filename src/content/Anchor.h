#ifndef ANCHOR_H
#define ANCHOR_H

#include "Options.h"
#include "io.h"

namespace slope {

class Anchor;
using AnchorPtr = std::shared_ptr<Anchor>;


class Anchor
{
public:
    virtual bool isPersistant() const {return false;}

    virtual void updatePos(const vec2& p) = 0;

    virtual vec2 getPos() const = 0;
};

class AbsoluteAnchor;
using AbsoluteAnchorPtr = std::shared_ptr<Anchor>;
class AbsoluteAnchor : public Anchor
{
protected:
    vec2 pos;
public:
    AbsoluteAnchor(const vec2& p) {pos = p;}

    static AbsoluteAnchorPtr Add(const vec2& p) {
        return std::make_shared<AbsoluteAnchor>(p);
    }
    virtual vec2 getPos() const override {
        return pos;
    }
    virtual void updatePos(const vec2 &p) override {
        pos = p;
    }
};

extern AbsoluteAnchorPtr GlobalAnchor;

class LabelAnchor;
using LabelAnchorPtr = std::shared_ptr<LabelAnchor>;
class LabelAnchor : public Anchor
{
protected:
    std::string label = "";
public:

    virtual bool isPersistant() const override { return true; }

    LabelAnchor(std::string l) : label(l) {
        writeAtLabel(0.5,0.5,false);
    }

    static LabelAnchorPtr Add(const std::string& l) {
        return std::make_shared<LabelAnchor>(l);
    }

    virtual vec2 getPos() const override {
        return GlobalAnchor->getPos() + readFromLabel();
    }

    virtual void updatePos(const vec2& p) override {
        std::cerr << "[WARNING] cannot change labeled anchor position by hand" << std::endl;
    }
    void writeAtLabel(double x, double y,bool overwrite) const;
    vec2 readFromLabel() const;
};

vec2 WorldToScreen(const vec& p);

vec ScreenToWorld(const vec2& p);

class DynamicAnchor : public Anchor
{
protected:
    std::function<vec2()> anchor;

    static std::function<vec2()> trackScreen(const std::function<vec()>& track) {
        return [track] () -> vec2 {
            return WorldToScreen(track());
        };
    }


    // Anchor interface
public:
    virtual void updatePos(const vec2 &p) override {}
    virtual vec2 getPos() const override {
        return anchor();
    }

    DynamicAnchor(std::function<vec2()> f) : anchor(f) {}

    static std::shared_ptr<DynamicAnchor> AddTracker(const std::function<vec()>& f) {
        return std::make_shared<DynamicAnchor>(trackScreen(f));
    }

    static std::shared_ptr<DynamicAnchor> Add(std::function<vec2()> f) {
        return std::make_shared<DynamicAnchor>(f);
    }
    static std::shared_ptr<DynamicAnchor> Add(const vec& x) {
        std::function<vec()> p = [x](){return x;};
        return AddTracker(p);
    }

};
}

#endif // ANCHOR_H
