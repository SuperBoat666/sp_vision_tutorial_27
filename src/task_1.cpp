#include <chrono>
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
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

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

      // 6. 装甲板相对云台的偏角（误差角）
      double yaw_rel   = target.ypr_in_gimbal[0];
      double pitch_rel = target.ypr_in_gimbal[1];

      // 7. 云台当前绝对角度（世界系）
      auto state = gimbal.state();
      double yaw_now   = state.yaw;
      double pitch_now = state.pitch;

      // 8. 绝对目标角度 = 当前绝对角度 + 误差角
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

      // 12. Plotter 输出
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

      // 13. 调试可视化
      cv::circle(img, target.center, 5, {0, 0, 255}, -1);
      cv::putText(
        img, "TRACKING", {10, 30}, cv::FONT_HERSHEY_SIMPLEX,
        1.0, {0, 255, 0}, 2);

    } else {
      // 丢失目标：不控制云台，让电控自己保持
      gimbal.send(false, false, 0.0f, 0.0f);

      nlohmann::json j;
      j["yaw_target"]   = 0.0;
      j["pitch_target"] = 0.0;
      j["yaw_now"]      = 0.0;
      j["pitch_now"]    = 0.0;
      j["yaw_cmd"]      = 0.0;
      j["pitch_cmd"]    = 0.0;
      j["lost"]         = true;
      plotter.plot(j);
    }

    // 14. 显示图像
    cv::imshow("auto_aim", img);
    cv::waitKey(1);
  }
  return 0;
}