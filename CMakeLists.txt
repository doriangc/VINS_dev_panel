cmake_minimum_required(VERSION 3.6)
project(imgui-opengl-glfw-glew-cmake-demo)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_VERBOSE_MAKEFILE 1)
set(APP_VERSION_MAJOR 1)
set(APP_VERSION_MINOR 0)

set(APP_TARGET dev_panel)

add_subdirectory(imgui)
include_directories(${PROJECT_SOURCE_DIR})
add_executable(${APP_TARGET} main.cpp binder/ViewController.cpp)
target_link_libraries(${APP_TARGET} imgui)

# glfw
find_package(glfw3 REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})
link_libraries(${GLFW_LIBRARY_DIRS})

# opengl
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# glew
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

add_subdirectory(VINS-Mobile-master)

if (APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OpenGL_LIBRARY OpenGL)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREVIDEO_LIBRARY CoreVideo)
    SET(EXTRA_LIBS ${COCOA_LIBRARY} ${OpenGL_LIBRARY} ${IOKIT_LIBRARY} ${COREVIDEO_LIBRARY})
endif (APPLE)

if (WIN32)
# nothing now
endif (WIN32)

target_link_libraries(
        ${APP_TARGET}
        VINSMobile
        VINS_Android
        glfw
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${EXTRA_LIBS}
)