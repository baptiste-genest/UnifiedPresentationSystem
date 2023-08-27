cmake_minimum_required(VERSION 3.5)

project(UPS LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(polyscope)

find_package(OpenGL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(UPS imgui_main.cpp
    UPS.h
    content/Image.h content/Image.cpp
    UnifiedPresentationSystem.h
    slides/Slideshow.h slides/Slideshow.cpp
    slides/Slide.h slides/Slide.cpp
    content/primitive.h
    math/kernels.h
    content/Text.h content/Text.cpp
    style/Style.h style/Style.cpp
    style/FontManager.h style/FontManager.cpp
    content/primitive.cpp
    content/PointCloud.h content/PointCloud.cpp
    content/LateX.h content/LateX.cpp
    content/io.h
    content/Mesh.h content/Mesh.cpp
    content/PolyscopePrimitive.h content/PolyscopePrimitive.cpp
    math/geometry.h
    content/Curve3D.h content/Curve3D.cpp
    content/Point.h content/Point.cpp
    content/TimeObject.h content/TimeObject.cpp
    content/Placement.h content/Placement.cpp
)
target_link_libraries(UPS polyscope OpenGL)

include(GNUInstallDirs)
install(TARGETS UPS
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
