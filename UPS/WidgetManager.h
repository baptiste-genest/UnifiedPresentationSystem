#ifndef WIDGETMANAGER_H
#define WIDGETMANAGER_H

#include "Widget.h"

namespace UPS {

class WidgetManager
{
public:
    WidgetManager();

    void run();

private:

    std::vector<WidgetPtr> widgets;

};

}

#endif // WIDGETMANAGER_H
