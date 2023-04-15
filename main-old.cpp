// ImGui - standalone example application for Glfw + OpenGL 3, using programmable pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.

#include <stdio.h>
#include <string>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/imgui_impl_glfw_gl3.h>

#include "binder/ViewController.hpp"

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}

int main(int, char**)
{
    // Setup window
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    float highDPIscaleFactor = 1.0;
    GLFWmonitor *monitor = glfwGetPrimaryMonitor();
    float xscale, yscale;
    glfwGetMonitorContentScale(monitor, &xscale, &yscale);
    if (xscale > 1 || yscale > 1)
    {
        highDPIscaleFactor = xscale;
        glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
    }

    GLFWwindow* window = glfwCreateWindow(1280, 720, "ImGui OpenGL3 example", NULL, NULL);
    glfwMakeContextCurrent(window);

    glewInit();
    
    const GLubyte *renderer = glGetString( GL_RENDERER );
    const GLubyte *vendor = glGetString( GL_VENDOR );
    const GLubyte *version = glGetString( GL_VERSION );
    const GLubyte *glslVersion = glGetString( GL_SHADING_LANGUAGE_VERSION );
    
    GLint major, minor, samples, sampleBuffers;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
	glGetIntegerv(GL_SAMPLES, &samples);
	glGetIntegerv(GL_SAMPLE_BUFFERS, &sampleBuffers);

	printf("-------------------------------------------------------------\n");
    printf("GL Vendor    : %s\n", vendor);
    printf("GL Renderer  : %s\n", renderer);
    printf("GL Version   : %s\n", version);
    printf("GL Version   : %d.%d\n", major, minor);
    printf("GLSL Version : %s\n", glslVersion);
	printf("MSAA samples : %d\n", samples);
	printf("MSAA buffers : %d\n", sampleBuffers);
    printf("-------------------------------------------------------------\n");
    
    // Setup ImGui binding
    ImGui_ImplGlfwGL3_Init(window, true);

    // Load FontsR
    // (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
    ImGuiIO& io = ImGui::GetIO();
    // io.Fonts->AddFontDefault();
    io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 15.0f * highDPIscaleFactor);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyClean.ttf", 13.0f);
    //io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyTiny.ttf", 10.0f);
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());

    bool show_test_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImColor(114, 144, 154);

    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(highDPIscaleFactor);

    int width = 480, height = 640;
    std::string filename = "/home/ubuntu/Downloads/VINS/18479.741080";

    FILE *file = fopen(filename.c_str(), "rb");

    fseek(file, 0, SEEK_END);          // Jump to the end of the file
    long filelen = ftell(file);        // Get the current byte offset in the file
    rewind(file);                      // Jump back to the beginning of the file

    int size;
    fread(&size, 1, sizeof(size), file);

    std::cout << "Measurement Size: " << size << std::endl;

    for (int i=0; i<size; i++) {
        double xAcc, yAcc, zAcc;
        double xGyr, yGyr, zGyr;

        fread(&xAcc, 1, sizeof(xAcc), file);
        fread(&yAcc, 1, sizeof(yAcc), file);
        fread(&zAcc, 1, sizeof(zAcc), file);

        fread(&xGyr, 1, sizeof(xGyr), file);
        fread(&xGyr, 1, sizeof(yGyr), file);
        fread(&xGyr, 1, sizeof(zGyr), file);
    }

    long curPos = ftell(file);
    long remaining = filelen - curPos;

    char* buffer = (char *)malloc(remaining * sizeof(char)); // Enough memory for the file
    fread(buffer, remaining, 1, file); // Read in the entire file
    fclose(file); // Close the file

    GLuint my_opengl_texture;
    glGenTextures(1, &my_opengl_texture);
    glBindTexture(GL_TEXTURE_2D, my_opengl_texture);  
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    glGenerateMipmap(GL_TEXTURE_2D);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplGlfwGL3_NewFrame();

        ImGuiViewport* viewport = ImGui::GetMainViewport();

        // if (ImGui::BeginMainMenuBar()) {
        //     if (ImGui::BeginMenu("Controls")) {
        //         ImGui::MenuItem("Play/Pause");
        //         ImGui::MenuItem("Next Frame");
        //         ImGui::MenuItem("Previous Frame");
        //         ImGui::EndMenu();
        //     }
        //     ImGui::EndMainMenuBar();
        // }

        // ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");

        // auto dockspace_id = ImGui::DockSpaceOverViewport();
        // ImGui::Begin("Docking WIndow", &show_another_window);
        // ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");

        // ImGuiID dockspace_id = ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

        // static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;

        // ImGui::DockBuilderRemoveNode(dockspace_id); // clear any previous layout
        // ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        // ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

        // ImGuiID top, bottom;

        // ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.9f, &top, &bottom);
        // // ImGui::DockBuilderDockWindow("Debug Window", bottom);
        // // ImGui::DockBuilderDockWindow("Dear ImGui Demo", top);
        // ImGui::DockBuilderFinish(dockspace_id);


        // 1. Show a simple window
        // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"
        {

       // [...] load image, render to texture, etc.
            ImGui::Begin("Debug Window", &show_another_window);
            ImGui::Image((void*)(intptr_t)my_opengl_texture, ImVec2(width,height));
            static float f = 0.0f;
            ImGui::Text("Hello, world!");
            // ImGui::ImageButton();
            // ImGui::Image();
            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
            ImGui::ColorEdit3("clear color", (float*)&clear_color);
            if (ImGui::Button("Test Window")) show_test_window ^= 1;
            if (ImGui::Button("Another Window")) show_another_window ^= 1;
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
        }


        // 2. Show another simple window, this time using an explicit Begin/End pair
        // ImGui::SetNextWindowSize(ImVec2(200,100), ImGuiCond_FirstUseEver);
        ImGui::Begin("Another Window", &show_another_window);
        ImGui::Text("Hello");
        // ImGui::End();
        ImGui::End();

        // 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
        if (show_test_window)
        {
            ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
            ImGui::ShowDemoWindow(&show_test_window);
        }

        // Rendering
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui::Render();
        ImGui_ImplGlfwGL3_RenderDrawLists(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();

    return 0;
}
