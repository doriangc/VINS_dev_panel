// Dear ImGui: standalone example application for SDL2 + OpenGL
// (SDL is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include "controller.hpp"
#include "imgui_setup.hpp"

#include <stdio.h>
#include <iostream>
#include <string>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>


// This example can also compile and run with Emscripten! See 'Makefile.emscripten' for details.
#ifdef __EMSCRIPTEN__
#include "../libs/emscripten/emscripten_mainloop_stub.h"
#endif

// Main code
int main(int, char**)
{
    Graphics graphics;
    graphics.setup();

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    
    // Main loop
    bool done = false;

    Controller controller;

    controller.loadPath("/home/ubuntu/Downloads/AT04");
    GLuint my_opengl_texture = controller.stepForwards();

    while (!done)
    {
        glDeleteTextures(1, &my_opengl_texture);
        my_opengl_texture = controller.stepForwards();

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)
                done = true;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::DockSpaceOverViewport(viewport);

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Controls");                          // Create a window called "Hello, world!" and append into it.

            // ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
            // ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            // ImGui::Checkbox("Another Window", &show_another_window);

            // ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            // ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            // if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
            //     counter++;
            // ImGui::SameLine();
            // ImGui::Text("counter = %d", counter);

            // ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }

        {
            ImGui::Begin("Viewport");
            float img_height = ImGui::GetWindowHeight() * 0.95;
            float img_width = 640.0f*(img_height/480.0f);

            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - img_width) * 0.5f, (ImGui::GetWindowHeight() - img_height) * 0.5f));
            ImGui::Image((void*)(intptr_t)my_opengl_texture, ImVec2(img_width, img_height));
            ImGui::End();
        }

        {
            ImGui::Begin("3D View");
            // float window_height = ImGui::GetWindowHeight();
            
            // ImGui::Image((void*)(intptr_t)my_opengl_texture, ImVec2(height*(window_height/width), window_height));
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        graphics.swap();
    }

    graphics.cleanup();

    LOGI("DONE!");

    return 0;
}
