#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_sdl2.h"
#include "imgui/backends/imgui_impl_opengl3.h"

class Graphics {
    SDL_Window* window;
    SDL_GLContext gl_context;
    ImGuiIO* io;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

public:
    Graphics() {}

    int setup();
    void swap();
    void cleanup();
};

