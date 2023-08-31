cmake_minimum_required(VERSION 3.5)

project(UPS LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(polyscope)

find_package(OpenGL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB SRC
     "content/*"
     "math/*"
     "slides/*"
     "style/*"
     "UnifiedPresentationSystem.h"
     "UPS.h"
)

add_subdirectory("../../libs/geometry-central" geometry-central)

add_executable(dev ../slides/DDG.cpp ${SRC})
target_link_libraries(dev polyscope OpenGL geometry-central)

add_executable(course_MG ../slides/course_MG.cpp ${SRC})
target_link_libraries(course_MG polyscope OpenGL geometry-central)
