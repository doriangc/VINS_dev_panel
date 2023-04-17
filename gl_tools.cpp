#include "gl_tools.hpp"

GLuint createGLTexture(void* buffer, uint width, uint height) {
    char* oBuffer = (char *)malloc(width * height * 4 * sizeof(char));

    // Rotate the image 90 degrees
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int src_index = (y * width + x) * 4;
            int dst_index = ((width - x - 1) * height + y) * 4;
            for (int c = 0; c < 4; c++) {
                oBuffer[dst_index + c] = ((char*)buffer)[src_index + c];
            }
        }
    }

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);  
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, height, width, 0, GL_RGBA, GL_UNSIGNED_BYTE, oBuffer);

    free(oBuffer);
    return texture;
}