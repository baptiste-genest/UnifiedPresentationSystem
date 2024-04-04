cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

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


## External tools configuration
find_package(LATEX COMPONENTS PDFLATEX REQUIRED)
find_program(UPS_CONVERT convert)
if (NOT UPS_CONVERT)
  message(ERROR "Could not find the 'convert' tool (imagemagick)")
endif()
set(UPS_PDFLATEX ${PDFLATEX_COMPILER})
set(UPS_SOURCE ${PROJECT_SOURCE_DIR})
message(STATUS "pdflatex compiler: ${UPS_PDFLATEX}")
message(STATUS "convert tool: ${UPS_CONVERT}")
message(STATUS "UPS source path: ${UPS_SOURCE}")
configure_file(${PROJECT_SOURCE_DIR}/src/content/Options.cpp.in
               ${PROJECT_SOURCE_DIR}/src/content/Options.cpp)


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

add_executable(minimal_surf projects/minimal_surf/main.cpp)
target_link_libraries(minimal_surf UPS)

## Tutorials
add_executable(tuto_helloworld projects/tutorials/tuto_helloworld.cpp)
target_link_libraries(tuto_helloworld UPS)

add_executable(tuto_mesh projects/tutorials/tuto_mesh.cpp)
target_link_libraries(tuto_mesh UPS)

add_executable(NESOTS projects/NESOTS/main.cpp)
target_link_libraries(NESOTS UPS)
