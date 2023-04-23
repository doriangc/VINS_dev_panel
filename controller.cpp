#include "controller.hpp"

#include <binder/ViewController.hpp>

#include <iostream>
#include <fstream>
#include <filesystem>

#include <opencv2/imgproc/types_c.h>

bool compare_entries(const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
    return a.path().filename().string() < b.path().filename().string();
}

Controller::Controller() {
    viewController = std::unique_ptr<ViewController>(new ViewController);
    LOGI("Successfully created Viewcontroller Object");

    viewController->viewDidLoad();
}

Controller::~Controller() {
    LOGI("Destructing Controller");
    reset();
    LOGI("Done with free");
}

void Controller::loadPath(std::string path) {
    uint width = 480, height = 640, bytesPerPix = 4;

    this->_frame_width = width;
    this->_frame_height = height;
    this->_currentFrame = -1;

    std::vector<std::filesystem::directory_entry> entries;

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (entry.is_regular_file()) {
            entries.push_back(entry);
        }
    }

    std::sort(entries.begin(), entries.end(), compare_entries);

    for (const auto & entry : entries) {
        
        FILE *file = fopen(entry.path().c_str(), "rb");

        int numMeasurements;
        fread(&numMeasurements, 1, sizeof(numMeasurements), file);

        std::vector<Measurement> imuFrame;

        for (int i=0; i<numMeasurements; i++) {
            double timestamp;
            double xAcc, yAcc, zAcc;
            double xGyr, yGyr, zGyr;

            fread(&timestamp, 1, sizeof(timestamp), file);
            fread(&xAcc, 1, sizeof(xAcc), file);
            fread(&yAcc, 1, sizeof(yAcc), file);
            fread(&zAcc, 1, sizeof(zAcc), file);

            fread(&xGyr, 1, sizeof(xGyr), file);
            fread(&yGyr, 1, sizeof(yGyr), file);
            fread(&zGyr, 1, sizeof(zGyr), file);

            imuFrame.push_back({timestamp, xAcc, yAcc, zAcc, xGyr, yGyr, zGyr});
        }
        timestamps.push_back(std::stod(entry.path().filename().string()));

        imuMeasurements.push_back(imuFrame);

        uint imageSize = width * height * bytesPerPix;
        char* buffer = (char *)malloc(imageSize * sizeof(char)); // Enough memory for the file
        fread(buffer, imageSize, 1, file); // Read in the entire image

        imageFrames.push_back(buffer);
        
        fclose(file); // Close the file
    }

    std::cout << "Done Loading " << path << std::endl;

}

GLuint Controller::stepForwards() {
    _currentFrame++;

    if (_currentFrame >= imageFrames.size()) return -1;

    std::vector<Measurement> curFrameImuMeasurements = imuMeasurements[_currentFrame];

    for (auto &measurement : curFrameImuMeasurements) {
        viewController->addMeasurement({
            measurement.timestamp,
            {measurement.xAcc, measurement.yAcc, measurement.zAcc},
            {measurement.xGyr, measurement.yGyr, measurement.zGyr},
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    cv::Mat rotatedRgba(_frame_height,_frame_width, CV_8UC4, imageFrames[_currentFrame]);

    viewController->virtualCamDistance = _virtualCamDistance;
    viewController->processImage(rotatedRgba, timestamps[_currentFrame], false);

    currentTexture = createGLTexture(rotatedRgba.data, _frame_width, _frame_height);
    return currentTexture;
}

GLuint Controller::stepBack() {
    return 0;
}

void Controller::reset() {
    for (char* frame : imageFrames) {
        free(frame);
    }

    imageFrames.clear();
}