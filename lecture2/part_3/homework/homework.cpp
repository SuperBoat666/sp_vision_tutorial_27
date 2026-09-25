#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 1. 读取../assets/demo.jpg
    cv::Mat img = cv::imread("/home/superboat/Desktop/sp_vision_tutorial_27/lecture2/part_3/assets/demo.jpg");
    if (img.empty()) {
        std::cout << "读取图像失败，请检查路径！" << std::endl;
        return -1;
    }

    // 使用 cvtColor 把图像转为灰度图（颜色空间：BGR2GRAY）
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 3. 使用 imwrite 把灰度图保存为 gray.jpg
    cv::imwrite("gray.jpg", gray);

    // 4. 在灰度图上用 circle 画一个圆，标记你要"瞄准"的位置
    cv::circle(gray, cv::Point(400, 300), 50, cv::Scalar(0), 3);

    // 5. 示灰度图，按任意键退出
    cv::imshow("Gray with Circle", gray);
    cv::waitKey(0); // 0 表示一直等待，直到按任意键

    return 0;
}