#include <chrono>
#include <iostream>
#include <optional>
#include <thread>
#include <cmath>
#include <cstring> // 用于 memset
#include <Eigen/Dense>

#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main()
{
  std::optional<auto_aim::Target> target;
  
  std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
  double sim_time = 0.0;
  
  double true_w = 2.5;          
  double current_angle = 0.0;   
  double radius = 0.2765;       

  std::cout << "--- 前哨站 EKF 纯离线测试开始 ---" << std::endl;
  std::cout << "正在模拟 2.5 rad/s 的旋转..." << std::endl;

  // 【暴力分配】预先分配一块与 Armor 大小相同的内存，并清零
  // 这比调用它的构造函数要安全得多，因为不会触发内部的 vector 初始化
  alignas(auto_aim::Armor) char armor_memory[sizeof(auto_aim::Armor)];
  std::memset(armor_memory, 0, sizeof(auto_aim::Armor));
  auto* armor = reinterpret_cast<auto_aim::Armor*>(armor_memory);

  while (sim_time < 10.0) { // 跑 10 秒钟自动结束
    std::this_thread::sleep_for(10ms);
    t += 10ms;
    sim_time += 0.01;
    
    current_angle += true_w * 0.01;
    if (current_angle > M_PI) current_angle -= 2 * M_PI;

    // 强制覆盖物理属性
    armor->name = auto_aim::ArmorName::outpost;
    
    // 直接给 Eigen 的 Vector3d 赋值 (由于已经被 memset 清零，内部不会有野指针)
    armor->xyz_in_world = Eigen::Vector3d(
        3.0 - radius * std::cos(current_angle), 
        - radius * std::sin(current_angle), 
        1.0
    );
    armor->ypr_in_world = Eigen::Vector3d(current_angle, 0, 0); 
    
    // 这里调用 xyz2ypd
    armor->ypd_in_world = tools::xyz2ypd(armor->xyz_in_world);

    try {
      // 3. 喂给 EKF 进行整车建模预测与更新
      if (!target.has_value()) {
        // 第一帧，初始化
        target = auto_aim::Target(*armor, t, radius, 3);
      } else {
        // 后续帧：预测 + 更新
        target->predict(t);
        target->update(*armor);
      }

      // 4. 检查滤波器状态并输出
      if (target.has_value() && target->is_converged()) {
        Eigen::VectorXd x = target->ekf_x();
        
        if (int(sim_time * 100) % 50 == 0) {
          std::cout << "时间: " << sim_time << "s | "
                    << "EKF_w: " << x[7] << " rad/s | "
                    << "EKF_r: " << x[8] << " m | "
                    << "EKF_l: " << x[9] << " m" << std::endl;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "发生异常: " << e.what() << std::endl;
      break;
    }
  }

  std::cout << "--- 测试结束 ---" << std::endl;
  return 0;
}