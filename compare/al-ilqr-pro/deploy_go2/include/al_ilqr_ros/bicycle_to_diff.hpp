#pragma once

#include <algorithm>
#include <cmath>

namespace al_ilqr_ros {

/**
 * @brief 差速驱动命令。
 */
struct DiffDriveCommand {
  double v_linear = 0.0;   // m/s
  double omega = 0.0;      // rad/s
};

/**
 * @brief 将自行车模型控制量 [accel, steering_angle] 映射为差速控制 [v, omega]。
 *
 * 映射关系：
 *   v_linear  = current_speed + accel * dt
 *   omega     = v_linear * tan(steering_angle) / virtual_wheelbase
 *
 * 并对输出进行饱和限幅，保证发送给下位机的指令在安全范围内。
 *
 * @param accel            纵向加速度 (m/s^2)
 * @param steering_angle   前轮转角 (rad)
 * @param current_speed    当前纵向速度 (m/s)
 * @param dt               控制周期 (s)
 * @param virtual_wheelbase  虚拟轴距，用于差速模型 (m)
 * @param max_v            最大线速度 (m/s)
 * @param max_omega        最大角速度 (rad/s)
 * @return DiffDriveCommand 差速命令
 */
inline DiffDriveCommand bicycleToDiff(double accel,
                                      double steering_angle,
                                      double current_speed,
                                      double dt,
                                      double virtual_wheelbase,
                                      double max_v,
                                      double max_omega) {
  DiffDriveCommand cmd;

  // 预测下一时刻线速度
  cmd.v_linear = current_speed + accel * dt;

  // 饱和限幅：线速度
  if (cmd.v_linear > max_v) {
    cmd.v_linear = max_v;
  } else if (cmd.v_linear < -max_v) {
    cmd.v_linear = -max_v;
  }

  // 防止除零，计算角速度
  if (std::abs(virtual_wheelbase) < 1e-6) {
    cmd.omega = 0.0;
  } else {
    cmd.omega = cmd.v_linear * std::tan(steering_angle) / virtual_wheelbase;
  }

  // 饱和限幅：角速度
  if (cmd.omega > max_omega) {
    cmd.omega = max_omega;
  } else if (cmd.omega < -max_omega) {
    cmd.omega = -max_omega;
  }

  return cmd;
}

/**
 * @brief 由目标线速度 + 前轮转角直接生成差速命令。
 *
 * 与 bicycleToDiff 的区别：线速度不再由
 * "当前 odom 速度 + accel*dt" 增量积分得到，而是直接采用规划轨迹
 * 中的前视速度（result.states[k](3)）。
 *
 * 动机（真机问题）：Go2 从静止起步时 odom 速度恒为 0，增量式
 * v = 0 + a*dt 每周期最多爬升 max_accel*dt（约 0.1 m/s），落在底盘
 * 死区内 → 机器人不动 → odom 速度保持 0 → 速度永远爬不出死区，
 * 形成自举死锁。改用规划轨迹前视速度后，起步指令直接达到规划的
 * 巡航爬升值（如 0.2-0.3 m/s），可靠脱离死区。
 *
 * @param target_speed     目标线速度 (m/s)，取自规划轨迹前视点
 * @param steering_angle   前轮转角 (rad)
 * @param virtual_wheelbase  虚拟轴距 (m)
 * @param max_v            最大线速度 (m/s)
 * @param max_omega        最大角速度 (rad/s)
 * @return DiffDriveCommand 差速命令
 */
inline DiffDriveCommand bicycleToDiffFromSpeed(double target_speed,
                                               double steering_angle,
                                               double virtual_wheelbase,
                                               double max_v,
                                               double max_omega) {
  DiffDriveCommand cmd;

  cmd.v_linear = std::clamp(target_speed, -max_v, max_v);

  if (std::abs(virtual_wheelbase) < 1e-6) {
    cmd.omega = 0.0;
  } else {
    cmd.omega = cmd.v_linear * std::tan(steering_angle) / virtual_wheelbase;
  }
  cmd.omega = std::clamp(cmd.omega, -max_omega, max_omega);

  return cmd;
}

}  // namespace al_ilqr_ros
