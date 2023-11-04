cmake_minimum_required(VERSION 3.5)

project(UPS LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CPM)
include(eigen)
include(polyscope)
include(geometry-central)
include(spdlog)

find_package(OpenGL REQUIRED)

file(GLOB SRC_lib
     "src/content/*"
     "src/content/Graphics2D/*"
     "src/math/*"
     "src/slides/*"
     "src/style/*"
     "src/UnifiedPresentationSystem.h"
     "src/UPS.h"
     "src/extern/svg.hpp"
)

## The library
add_library(UPS ${SRC_lib})
target_link_libraries(UPS PUBLIC polyscope Eigen3::Eigen spdlog  OpenGL::GL geometry-central)

## Slideshows
add_executable(dev projects/dev/main.cpp)
target_link_libraries(dev UPS)

add_executable(course_MG projects/course_MG/main.cpp)
target_link_libraries(course_MG UPS)

## Tutorials
add_executable(tuto_helloworld projects/tutorials/tuto_helloworld.cpp)
target_link_libraries(tuto_helloworld UPS)

add_executable(tuto_mesh projects/tutorials/tuto_mesh.cpp)
target_link_libraries(tuto_mesh UPS)
