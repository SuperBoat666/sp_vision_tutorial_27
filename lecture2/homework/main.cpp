#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、yolo类
    Camera cam;
    auto_aim::YOLO yolo("configs/yolo.yaml");

    while (1) {
        // 调用相机读取图像
        cv::Mat img;
        if (!cam.read(img)) break;

        // 调用yolo识别装甲板
        auto armors = yolo.detect(img);

    
        for (const auto& armor : armors) {
        // 1. 绿色矩形
        tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);

        // 2. 文字
        std::string color_str = auto_aim::COLORS[armor.color];      // "blue" / "red"
        std::string name_str  = auto_aim::ARMOR_NAMES[armor.name];  // "one" / "two" / ... / "four"

        std::string label = color_str + name_str;  // "bluefour"

        cv::Point2f text_pos = armor.points[0];    // 
        cv::putText(img, label, text_pos,
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2);     // 
    }

        // 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}