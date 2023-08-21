#include "WidgetManager.h"

void UPS::WidgetManager::run()
{

    for (auto widget : widgets){
        ImGui::Begin(widget->getLabel().c_str());

        widget->run();

        ImGui::End();
    }

}
