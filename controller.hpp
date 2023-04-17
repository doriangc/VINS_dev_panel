#pragma once
#include "binder/ViewController.hpp"
#include "gl_tools.hpp"

#include <string>
#include <vector>

struct Measurement {
    double timestamp;
    double xAcc, yAcc, zAcc;
    double xGyr, yGyr, zGyr;
};

class Controller {
public:
    Controller();
    ~Controller();

    void loadPath(std::string path);

    GLuint stepForwards();
    GLuint stepBack();

    void reset();
private:
    uint _currentFrame, _frame_width, _frame_height;
    float _virtualCamDistance = 2.0f;
    float _minVirtualCamDistance = 2.0f;
    float _maxVirtualCamDistance = 40.0f;
    GLuint currentTexture;

    std::unique_ptr<ViewController> viewController;

    std::vector<double> timestamps;
    std::vector<char*> imageFrames;
    std::vector<std::vector<Measurement>> imuMeasurements;
};