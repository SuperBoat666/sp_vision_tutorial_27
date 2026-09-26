#pragma once

#include <opencv2/opencv.hpp>

class CameraTest {
public:
    CameraTest();
    ~CameraTest();
    bool read(cv::Mat& img);

private:
    cv::VideoCapture cap_;
};