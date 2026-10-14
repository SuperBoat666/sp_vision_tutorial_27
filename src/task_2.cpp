#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
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

  // ===== 初始化工具类 =====
  tools::Exiter exiter;
  tools::Plotter plotter;

  // ===== 初始化 io 类 =====
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // ===== 初始化 auto_aim 类 =====
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  // ===== 图像/姿态缓冲 =====
  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  // ===== Task_2 常量 =====
  constexpr double BULLET_SPEED  = 20.0;                  // 弹速 m/s，实测标定值（改动1：注释）
  constexpr double FIRE_INTERVAL = 0.2;                   // 5Hz → 0.2s
  constexpr int    MAX_FIRE      = 10;                    // 每挡位最多10发
  constexpr double YAW_TOL       = 0.5 * CV_PI / 180.0;   // 开火 yaw 阈值
  constexpr double PITCH_TOL     = 0.5 * CV_PI / 180.0;   // 开火 pitch 阈值

  // ===== Task_2 状态 =====
  std::unique_ptr<auto_aim::Target> target;
  auto last_fire_time = std::chrono::steady_clock::now();
  int fire_count  = 0;
  int last_gear   = -1;
  int frame_count = 0;

  while (!exiter.exit()) {
    // ===== 1. 采集图像 & 姿态 =====
    camera.read(img, t);
    q = gimbal.q(t);

    // ===== 2. 姿态注入 Solver =====
    solver.set_R_gimbal2world(q);

    // ===== 3. YOLO 检测 =====
    auto armors = yolo.detect(img, frame_count++);

    // ===== 4. 解算世界坐标 =====
    for (auto & armor : armors) solver.solve(armor);

    // ===== 5. Target 更新 =====
    if (!armors.empty()) {
      if (!target) {
        target = std::make_unique<auto_aim::Target>(armors.front(), t);
      } else {
        target->predict(t);
        // 选与 EKF 旋转中心最近的一块装甲板
        Eigen::VectorXd x_now = target->ekf_x();
        Eigen::Vector3d c(x_now[0], x_now[2], x_now[4]);
        auto best_it = armors.begin();
        double best_d = 1e10;
        for (auto it = armors.begin(); it != armors.end(); ++it) {
          double d = (it->xyz_in_world - c).norm();
          if (d < best_d) { best_d = d; best_it = it; }
        }
        target->update(*best_it);
      }
    } else if (target) {
      target->predict(t);
    }

    // ===== 6. 云台控制 + 开火 =====
    if (target && target->is_converged()) {
      Eigen::VectorXd x = target->ekf_x();
      double a = x[6];   // 当前旋转角
      double w = x[7];   // 角速度 (rad/s)
      double r = x[8];   // 半径

      // 6.1 选离相机最近的装甲板 id
      auto xyza_list = target->armor_xyza_list();
      int best_id = 0;
      double best_d = 1e10;
      for (int i = 0; i < static_cast<int>(xyza_list.size()); i++) {
        double d = tools::xyz2ypd(xyza_list[i].head(3))[2];
        if (d < best_d) { best_d = d; best_id = i; }
      }

      // 6.2 弹丸飞行时间
      double t_fly = best_d / BULLET_SPEED;

      // 6.3 提前量补偿：预测 t_fly 后装甲板位置
      double a_pred = a + w * t_fly + best_id * 2 * CV_PI / 4.0;
      Eigen::Vector3d pred_armor = {
        x[0] - r * std::cos(a_pred),
        x[2] - r * std::sin(a_pred),
        x[4]
      };
      Eigen::Vector3d ypd = tools::xyz2ypd(pred_armor);
      double target_yaw   = ypd[0];
      double target_pitch = ypd[1];

      // 6.4 转速挡位判定 & 计数复位
      double abs_w = std::abs(w);
      int gear = -1;
      if (abs_w >= 3.0 && abs_w < 5.0) gear = 0;        // 低转速
      else if (abs_w >= 5.0 && abs_w <= 9.0) gear = 1;  // 中转速

      if (gear != -1 && gear != last_gear) {
        fire_count = 0;
        last_gear = gear;
        tools::logger()->info("[Task2] gear -> {}", gear == 0 ? "LOW" : "MID");
      }

      // 6.5 开火条件
      auto gs = gimbal.state();
      auto now = std::chrono::steady_clock::now();
      double dt_fire = tools::delta_time(now, last_fire_time);
      double yaw_err   = std::abs(tools::limit_rad(gs.yaw   - target_yaw));
      double pitch_err = std::abs(tools::limit_rad(gs.pitch - target_pitch));

      bool can_fire = (gear != -1) && (fire_count < MAX_FIRE) &&
                      (dt_fire >= FIRE_INTERVAL) &&
                      (yaw_err < YAW_TOL) && (pitch_err < PITCH_TOL);

      gimbal.send(
        true, can_fire,
        static_cast<float>(target_yaw),
        static_cast<float>(target_pitch));

      if (can_fire) {
        last_fire_time = now;
        fire_count++;
        tools::logger()->info("[Task2] fire {}/{}", fire_count, MAX_FIRE);
      }

      // 6.6 Plotter 输出（任务书第4点要求）
      nlohmann::json plot_data;
      plot_data["w_ekf"]     = w;              // EKF 拟合角速度
      plot_data["yaw_cmd"]   = target_yaw;     // 云台 yaw 控制命令
      plot_data["pitch_cmd"] = target_pitch;   // 云台 pitch 控制命令
      plotter.plot(plot_data);
    }

    // ===== 7. 调试显示 =====
    if (!img.empty()) {
      cv::imshow("auto_aim", img);
      cv::waitKey(1);
    }
  }

  return 0;
}