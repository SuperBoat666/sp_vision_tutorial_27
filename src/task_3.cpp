#include <chrono>
#include <iostream>
#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

  // 前哨站整车追踪目标 (optional 用于延迟初始化)
  std::optional<auto_aim::Target> target;

  // 保存上一帧控制量，丢失目标时保持
  float last_yaw_cmd   = 0.0f;
  float last_pitch_cmd = 0.0f;

  while (!exiter.exit()) {
    // 1. 获取图片与云台姿态
    camera.read(img, t);
    q = gimbal.q(t);

    // 2. 更新云台姿态到 Solver (非常重要)
    solver.set_R_gimbal2world(q);

    // 3. YOLO 识别
    auto armors = yolo.detect(img);
    
    // 4. 寻找前哨站并更新整车 EKF
    bool found_outpost = false;
    for (auto & armor : armors) {
      if (armor.name == auto_aim::ArmorName::outpost) {
        found_outpost = true;
        
        // 【核心】先对当前这块单板进行 PnP 解算，获取其世界坐标
        solver.solve(armor);

        // 喂给整车模型 (EKF)
        if (!target.has_value()) {
          // 第一次看到前哨站，初始化模型 (半径 0.2765，装甲板数量 3)
          target = auto_aim::Target(armor, t, 0.2765, 3);
        } else {
          // 后续帧：利用 EKF 预测和更新，这就是整车建模的精髓！
          // predict 会根据匀速旋转模型预测整车状态
          // update 会利用当前单板观测，修正中心位置、转速 w 以及高度差 l
          target->predict(t);
          target->update(armor);
        }
        break; // 找到一块板就跳出循环，一帧只需更新一次 EKF
      }
    }

    // 5. 如果目标存在且 EKF 收敛 (未发散)
    if (target.has_value() && target->is_converged()) {
      Eigen::VectorXd x = target->ekf_x();
      
      // 获取当前云台姿态
      auto state = gimbal.state();
      double yaw_now   = state.yaw;
      double pitch_now = state.pitch;

      // 获取 EKF 预测出的三块装甲板的世界坐标列表 (xyza)
      auto xyza_list = target->armor_xyza_list();
      
      // 选择离当前云台 Yaw 最近的一块板，减少云台转动幅度
      double min_diff = 1e10;
      Eigen::Vector3d best_target_xyz;
      
      for (int i = 0; i < 3; i++) {
        // 取出 xyz 坐标 (Eigen 维度必须是 <3>)
        Eigen::Vector3d armor_xyz = xyza_list[i].head<3>();
        
        // 计算世界坐标系下的相对偏角
        double armor_yaw = std::atan2(armor_xyz.y(), armor_xyz.x());
        double diff = std::abs(tools::limit_rad(armor_yaw - yaw_now));
        
        if (diff < min_diff) {
          min_diff = diff;
          best_target_xyz = armor_xyz;
        }
      }

      // ================= 击打逻辑 =================
      // 将预测的世界坐标转为云台局部坐标系 (沿用 Task 1 的优秀逻辑)
      Eigen::Vector3d p_gimbal = q.inverse() * best_target_xyz;

      double x_g = p_gimbal[0];
      double y_g = p_gimbal[1];
      double z_g = p_gimbal[2];

      // 计算相对 Yaw 与相对 Pitch
      double yaw_rel = std::atan2(y_g, x_g);
      
      // 补偿重力 (弹速直接从云台读取)
      float bullet_speed = state.bullet_speed;
      if (bullet_speed < 10.0f) bullet_speed = 20.0f; 
      
      double distance = std::sqrt(x_g*x_g + y_g*y_g + z_g*z_g);
      double fly_time = distance / bullet_speed;
      double gravity_drop = 0.5 * 9.8 * fly_time * fly_time;
      
      // 补偿下坠
      double pitch_rel = std::atan2(z_g + gravity_drop, std::sqrt(x_g*x_g + y_g*y_g));

      // 计算云台绝对目标角度
      double yaw_target   = yaw_now + yaw_rel;
      double pitch_target = pitch_now + pitch_rel;

      // Yaw 限幅 [-pi, pi]
      yaw_target = tools::limit_rad(yaw_target);

      // 判断是否允许开火（相对角度差足够小即可，防止甩狙打空）
      bool fire = (std::abs(yaw_rel) < 0.05 && std::abs(pitch_rel) < 0.05);

      // 发送指令 (true: 控制云台, fire: 开火)
      gimbal.send(true, fire, (float)yaw_target, (float)pitch_target);

      // 保存控制量
      last_yaw_cmd   = (float)yaw_target;
      last_pitch_cmd = (float)pitch_target;

      // ================= Plotter 输出 =================
      // Task 4 考核要求：输出 EKF 拟合的角速度 w (x[7])
      nlohmann::json j;
      j["w"] = x[7];             
      j["yaw_cmd"] = yaw_target;
      j["pitch_cmd"] = pitch_target;
      plotter.plot(j);

    } else {
      // 6. 丢失目标或未收敛
      // 保持上一帧角度，防止云台乱抖
      gimbal.send(true, false, last_yaw_cmd, last_pitch_cmd);

      nlohmann::json j;
      j["lost"] = true;
      j["w"] = 0.0; // 丢失目标时给 Plotter 画个 0
      plotter.plot(j);
    }
  }

  return 0;
}