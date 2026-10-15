#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 初始化 io 类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化 auto_aim 类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // 保存上一帧控制量，丢失目标时保持
  float last_yaw_cmd   = 0.0f;
  float last_pitch_cmd = 0.0f;

  while (!exiter.exit()) {

    // 1. 获取图片
    camera.read(img, t);

    // 2. 获取云台四元数
    q = gimbal.q(t);

    // 3. solver + YOLO
    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);

    // 4. 选目标：离图像中心最近的
    if (!armors.empty()) {
      cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);

      auto target_it = std::min_element(
        armors.begin(), armors.end(),
        [&](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          auto da = a.center - img_center;
          auto db = b.center - img_center;
          return da.dot(da) < db.dot(db);
        });

      auto & target = *target_it;

      // 5. 解算 target
      solver.solve(target);

      // 6. 用 xyz_in_gimbal 算云台需要转的角度
      //    云台坐标系：X 前（子弹方向）、Y 左、Z 上
      double x = target.xyz_in_gimbal[0];
      double y = target.xyz_in_gimbal[1];
      double z = target.xyz_in_gimbal[2];

      // 试法 1：yaw 用 y 的正号
      double yaw_rel   = std::atan2(y, x);
      double pitch_rel = std::atan2(z, std::sqrt(x*x + y*y));

      // 7. 云台当前世界系角度
      auto state = gimbal.state();
      double yaw_now   = state.yaw;
      double pitch_now = state.pitch;

      // 8. 世界系绝对目标角度 = 当前角度 + 相对偏角
      double yaw_target   = yaw_now + yaw_rel;
      double pitch_target = pitch_now + pitch_rel;

      // 9. yaw 限幅到 [-pi, pi]
      while (yaw_target >  M_PI) yaw_target -= 2 * M_PI;
      while (yaw_target < -M_PI) yaw_target += 2 * M_PI;

      // 10. 转成 float
      float yaw_cmd   = static_cast<float>(yaw_target);
      float pitch_cmd = static_cast<float>(pitch_target);

      // 11. 发送给云台（绝对角度）
      gimbal.send(true, false, yaw_cmd, pitch_cmd);

      // 保存本次控制量
      last_yaw_cmd   = yaw_cmd;
      last_pitch_cmd = pitch_cmd;

      // // 12. 打印诊断（调试用，正式提交可注释）
      // std::cout << "x=" << x << " y=" << y << " z=" << z
      //           << " | yaw_rel=" << yaw_rel
      //           << " pitch_rel=" << pitch_rel << std::endl;

      // 13. Plotter 输出
      nlohmann::json j;
      j["yaw_now"]      = yaw_now;
      j["yaw_rel"]      = yaw_rel;
      j["yaw_target"]   = yaw_target;
      j["pitch_now"]    = pitch_now;
      j["pitch_rel"]    = pitch_rel;
      j["pitch_target"] = pitch_target;
      j["yaw_cmd"]      = yaw_cmd;
      j["pitch_cmd"]    = pitch_cmd;
      plotter.plot(j);

      // // 14. 调试可视化（正式提交可注释）
      // cv::circle(
      //   img, cv::Point(static_cast<int>(target.center.x),
      //                  static_cast<int>(target.center.y)),
      //   5, cv::Scalar(0, 0, 255), -1);
      // cv::putText(
      //   img, "TRACKING", cv::Point(10, 30),
      //   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    } else {
      // 丢失目标：保持上一帧角度，云台不跳
      gimbal.send(true, false, last_yaw_cmd, last_pitch_cmd);

      nlohmann::json j;
      j["lost"] = true;
      plotter.plot(j);
    }

    // // 15. 显示图像（正式提交可注释）
    // cv::imshow("auto_aim", img);
    // cv::waitKey(1);
  }

  return 0;
}