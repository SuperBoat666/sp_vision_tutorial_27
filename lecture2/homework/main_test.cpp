#include "io/camera_test.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、yolo类
    CameraTest cam;
    auto_aim::YOLO yolo("configs/yolo.yaml");

    while (1) {
        // 调用相机读取图像
        cv::Mat img;
        if (!cam.read(img)) break;

        // 调用yolo识别装甲板
        auto armors = yolo.detect(img);

        // 画矩形 + 文字
        for (const auto& armor : armors) {
            // 1. 绿色闭合矩形
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);

            // 2. 文字：颜色 + 数字
            std::string label = auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];
            cv::putText(img, label, armor.points[0],
                        cv::FONT_HERSHEY_SIMPLEX, 1.0,
                        cv::Scalar(0, 0, 255), 2);
        }

        // 显示图像
        cv::resize(img, img, cv::Size(800, 600));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}