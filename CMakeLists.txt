cmake_minimum_required(VERSION 3.5)

project(UPS LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(polyscope)
include(geometry-central)

find_package(OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB SRC
     "src/content/*"
     "src/content/Graphics2D/*"
     "src/math/*"
     "src/slides/*"
     "src/style/*"
     "src/UnifiedPresentationSystem.h"
     "src/UPS.h"
     "src/extern/svg.hpp"
)

add_executable(dev projects/dev/main.cpp ${SRC})
target_link_libraries(dev polyscope OpenGL::GL geometry-central)

add_executable(course_MG projects/course_MG/main.cpp ${SRC})
target_link_libraries(course_MG polyscope OpenGL::GL geometry-central)
