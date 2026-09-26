#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class Camera {
public:
    Camera();                  // 构造函数
    ~Camera();                 // 析构函数
    bool read(cv::Mat& img);   // 读取

private:
    void* handle_;             
};