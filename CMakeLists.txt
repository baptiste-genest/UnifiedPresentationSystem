cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(slope LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CPM)
include(eigen)
include(polyscope)
include(geometry-central)
include(spdlog)

find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)
pkg_check_modules(RSVG REQUIRED librsvg-2.0)
pkg_check_modules(GDKPIXBUF REQUIRED gdk-pixbuf-2.0)


## External tools configuration
find_package(LATEX COMPONENTS PDFLATEX REQUIRED)
find_program(Slope_CONVERT convert)
if (NOT Slope_CONVERT)
  message(ERROR "Could not find the 'convert' tool (imagemagick)")
endif()
set(Slope_PDFLATEX ${PDFLATEX_COMPILER})
set(Slope_SOURCE ${PROJECT_SOURCE_DIR})
message(STATUS "pdflatex compiler: ${Slope_PDFLATEX}")
message(STATUS "convert tool: ${Slope_CONVERT}")
message(STATUS "Slope source path: ${Slope_SOURCE}")
configure_file(${PROJECT_SOURCE_DIR}/src/content/Options.cpp.in
               ${PROJECT_SOURCE_DIR}/src/content/Options.cpp)


file(GLOB SRC_lib
     "src/content/*"
     "src/content/Graphics2D/*"
     "src/content/plots/*"
     "src/math/*"
     "src/slides/*"
     "src/style/*"
     "src/Slope.h"
     "src/UPS.h"
     "src/extern/*"
)
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "src/*.h*")

## The library
add_library(slope STATIC ${SRC_lib})
target_link_libraries(slope PUBLIC polyscope Eigen3::Eigen spdlog  OpenGL::GL geometry-central ${CAIRO_LIBRARIES} ${RSVG_LIBRARIES} ${RSVG_LDFLAGS} ${GDKPIXBUF_LIBRARIES})
target_compile_features(slope PUBLIC cxx_std_20)
target_include_directories(slope PUBLIC src/ PRIVATE ${CAIRO_INCLUDE_DIRS} ${RSVG_INCLUDE_DIRS})
include_directories(
        ${CAIRO_INCLUDE_DIRS}
        ${RSVG_INCLUDE_DIRS}
        ${GDKPIXBUF_INCLUDE_DIRS}
)
link_directories(slope PRIVATE ${CAIRO_LIBRARY_DIRS} ${RSVG_LIBRARY_DIRS})
target_compile_options(slope PRIVATE ${CAIRO_CFLAGS_OTHER})
set_target_properties(slope PROPERTIES PUBLIC_HEADER "${HEADER_LIST}")

##INSTALL
install(TARGETS slope
        EXPORT slope
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        PUBLIC_HEADER DESTINATION include/slope
        )


option(BUILD_slope_EXAMPLES "Build the slope examples" ON)

if (BUILD_slope_EXAMPLES)
## Slideshows
add_executable(dev projects/dev/main.cpp)
target_link_libraries(dev slope)

#add_executable(course_MG projects/course_MG/main.cpp)
#target_link_libraries(course_MG slope)

add_executable(minimal_surf projects/minimal_surf/main.cpp)
target_link_libraries(minimal_surf slope)

## Tutorials
add_executable(tuto_helloworld projects/tutorials/tuto_helloworld.cpp)
target_link_libraries(tuto_helloworld slope)

add_executable(tuto_mesh projects/tutorials/tuto_mesh.cpp)
target_link_libraries(tuto_mesh slope)

add_executable(NESOTS projects/NESOTS/main.cpp)
target_link_libraries(NESOTS slope)

file(GLOB implicit_uvs_slides
     "projects/implicit_uvs/slides/*"
)
add_executable(implicit_uvs projects/implicit_uvs/main.cpp ${implicit_uvs_slides})
target_link_libraries(implicit_uvs slope)

file(GLOB implicit_uvs_slides_simple
     "projects/implicit_uvs_simple/slides/*"
)
add_executable(implicit_uvs_simple projects/implicit_uvs_simple/main.cpp ${implicit_uvs_slides_simple})
target_link_libraries(implicit_uvs_simple slope)

endif()


