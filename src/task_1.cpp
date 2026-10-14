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

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // 上一帧控制量（丢失目标时保持）
  float last_yaw_cmd   = 0.0f;
  float last_pitch_cmd = 0.0f;

  while (!exiter.exit()) {

    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);

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
      solver.solve(target);

      double yaw_rel   = target.ypr_in_gimbal[0];
      double pitch_rel = target.ypr_in_gimbal[1];

      auto state = gimbal.state();
      double yaw_now   = state.yaw;
      double pitch_now = state.pitch;

      // 绝对目标角度（版本 A）
      // 如果方向反了，把 + 改成 -
      double yaw_target   = yaw_now + yaw_rel;
      double pitch_target = pitch_now + pitch_rel;

      while (yaw_target >  M_PI) yaw_target -= 2 * M_PI;
      while (yaw_target < -M_PI) yaw_target += 2 * M_PI;

      float yaw_cmd   = static_cast<float>(yaw_target);
      float pitch_cmd = static_cast<float>(pitch_target);

      gimbal.send(true, false, yaw_cmd, pitch_cmd);

      last_yaw_cmd   = yaw_cmd;
      last_pitch_cmd = pitch_cmd;

      // 打印诊断（调试用，正式提交可注释）
      std::cout << "yaw_now=" << yaw_now
                << " yaw_rel=" << yaw_rel
                << " yaw_target=" << yaw_target << std::endl;

      nlohmann::json j;
      j["yaw_rel"]      = yaw_rel;
      j["pitch_rel"]    = pitch_rel;
      j["yaw_now"]      = yaw_now;
      j["pitch_now"]    = pitch_now;
      j["yaw_target"]   = yaw_target;
      j["pitch_target"] = pitch_target;
      j["yaw_cmd"]      = yaw_cmd;
      j["pitch_cmd"]    = pitch_cmd;
      plotter.plot(j);

      // 可视化（正式提交可注释）
      cv::circle(img, target.center, 5, {0, 0, 255}, -1);
      cv::putText(img, "TRACKING", {10, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    } else {
      // 丢失目标：保持上一帧角度
      gimbal.send(true, false, last_yaw_cmd, last_pitch_cmd);

      nlohmann::json j;
      j["lost"] = true;
      plotter.plot(j);
    }

    // 显示图像（正式提交可注释）
    cv::imshow("auto_aim", img);
    cv::waitKey(1);
  }

  return 0;
}