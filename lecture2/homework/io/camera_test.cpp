#include "io/camera_test.hpp"

CameraTest::CameraTest() {
    cap_.open(0);
    if (!cap_.isOpened()) {
        throw std::runtime_error("CameraTest: cannot open webcam");
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
}

CameraTest::~CameraTest() {
    if (cap_.isOpened()) cap_.release();
}

bool CameraTest::read(cv::Mat& img) {
    return cap_.read(img);
}