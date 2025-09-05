#include "PolyscopePrimitive.h"

size_t slope::PolyscopePrimitive::count = 0;
std::vector<glm::vec3>  slope::PolyscopePrimitive::colors = {};
int slope::PolyscopePrimitive::current_color_id = 0;

#include "polyscope/color_management.h"

glm::vec3 slope::PolyscopePrimitive::getColor() {
    if (colors.size() == 0) {
        for (int i = 0;i<10;i++)
            colors.push_back(polyscope::getNextUniqueColor());
        current_color_id = 0;
    }
    return colors[(current_color_id++)%colors.size()];
}