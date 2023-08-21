#ifndef WIDGET_H
#define WIDGET_H

#include "UPS.h"

namespace UPS {

class Widget
{
public:
    Widget(std::string& label);

    virtual void run() {}

    inline std::string getLabel() const {return label;}

private:
    std::string label;

};

}

#endif // WIDGET_H
