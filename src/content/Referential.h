#ifndef REFERENTIAL_H
#define REFERENTIAL_H

#include "../UPS.h"
#include "Options.h"
#include "io.h"

namespace UPS {

class Slide;

struct Position {
    virtual vec2 getPosition(const Slide&) const = 0;
    virtual void writeAtLabel(scalar,scalar,bool) const {}
    virtual bool isPersistant() const { return false; }
};

using PositionPtr = std::shared_ptr<Position>;

struct AbsolutePosition : public Position {
    vec2 p;
    AbsolutePosition(const vec2& p) : p(p) {}
    virtual vec2 getPosition(const Slide&) const override {
        return p;
    }
};

class ScreenPrimitive;
using RelativePlacer = std::function<vec2(vec2)>;

struct RelativePosition : public Position {
    std::shared_ptr<ScreenPrimitive> ref;
    RelativePlacer placement;

    virtual vec2 getPosition(const Slide&) const override;
    RelativePosition(std::shared_ptr<ScreenPrimitive> ref,RelativePlacer placement) : ref(ref),placement(placement) {}
};

struct PersistantPosition : public Position {
    std::string label;

    PersistantPosition(std::string label) : label(label) {}

    void setLabel(std::string label);
    void writeAtLabel(double x,double y,bool overwrite) const override;
    vec2 readFromLabel() const;
    virtual bool isPersistant() const override { return true; }

    virtual vec2 getPosition(const Slide&) const override {
        return readFromLabel();
    }
};


}

#endif // REFERENTIAL_H
