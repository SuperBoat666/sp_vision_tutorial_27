#include "io/camera_test.hpp"
#include "tasks/apriltag_detector.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、检测器
    CameraTest cam;
    auto_charge::AprilTagDetector detector("configs/yolo.yaml");

    while (1) {
        // 调用相机读取图像
        cv::Mat img;
        if (!cam.read(img)) break;

        // 检测 AprilTag
        auto tags = detector.detect(img);

        // 画框 + ID
        for (const auto& tag : tags) {
            // 1. 绿色闭合矩形
            tools::draw_points(img, tag.corners, cv::Scalar(0, 255, 0), 2);

            // 2. 显示 ID
            cv::putText(img, std::to_string(tag.id), tag.center,
                        cv::FONT_HERSHEY_SIMPLEX, 1.0,
                        cv::Scalar(0, 0, 255), 2);
        }

        // 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}