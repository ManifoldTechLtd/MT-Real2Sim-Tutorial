#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

#include "autodrive/global_guidance_planner.hpp"

namespace al_ilqr_ros {

/**
 * @brief 将 nav_msgs::Path 转换为 AL-iLQR GuidancePoint 序列。
 *
 * 假设输入 path 已经位于规划坐标系（通常为 odom）。处理流程：
 *   1. 在路径上找到离机器人最近的点；
 *   2. 从该点向前按等弧长间隔 dt*ref_speed 采样 horizon+1 个点；
 *   3. 由相邻点方向计算航向角 yaw；
 *   4. 由三点拟合近似曲率，并根据曲率调整参考速度（弯道减速）；
 *   5. 计算每个 guidance point 的累计弧长 s、时间 t 和横向偏移 d。
 */
class PathConverter {
 public:
  using GuidancePoint = my_al_ilqr::GuidancePoint;

  PathConverter() = default;

  /**
   * @brief 配置采样参数。
   *
   * @param horizon   预测时域步数
   * @param dt        单步时长 (s)
   * @param ref_speed 默认参考速度 (m/s)
   */
  void configure(int horizon, double dt, double ref_speed) {
    horizon_ = horizon;
    dt_ = dt;
    ref_speed_ = ref_speed;
  }

  /**
   * @brief 设置弯道降速参数。
   *
   * @param curvature_weight  曲率对速度的衰减权重，越大减速越明显
   * @param min_speed         允许的最小参考速度 (m/s)
   * @param max_speed         允许的最大参考速度 (m/s)
   */
  void configureSpeedScaling(double curvature_weight,
                             double min_speed,
                             double max_speed) {
    curvature_weight_ = curvature_weight;
    min_speed_ = min_speed;
    max_speed_ = max_speed;
  }

  /**
   * @brief 从全局路径中提取 guidance points。
   *
   * @param global_path    输入全局路径
   * @param robot_x        机器人当前 x (m)
   * @param robot_y        机器人当前 y (m)
   * @param robot_yaw      机器人当前航向 (rad)
   * @param current_speed  机器人当前速度 (m/s)
   * @param reached_end    可选输出：若非空，当采样窗口触及路径末端
   *                       （存在被夹到路径终点的采样点）时置 true，
   *                       可用于辅助到达判定
   * @return std::vector<GuidancePoint> guidance 序列；若路径无效则返回空。
   */
  std::vector<GuidancePoint> extractGuidance(const nav_msgs::Path& global_path,
                                             double robot_x,
                                             double robot_y,
                                             double robot_yaw,
                                             double current_speed,
                                             bool* reached_end = nullptr) {
    (void)robot_yaw;
    (void)current_speed;

    if (reached_end != nullptr) {
      *reached_end = false;
    }

    std::vector<GuidancePoint> guidance;

    const auto& poses = global_path.poses;
    if (poses.size() < 2) {
      return guidance;
    }

    const int n = static_cast<int>(poses.size());

    // 计算累计弧长
    std::vector<double> arc_length(n, 0.0);
    for (int i = 1; i < n; ++i) {
      const double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
      const double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
      arc_length[i] = arc_length[i - 1] + std::hypot(dx, dy);
    }

    const double total_length = arc_length.back();
    if (total_length < 1e-6) {
      return guidance;
    }

    // 找到离机器人最近的点索引
    int closest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
      const double dx = poses[i].pose.position.x - robot_x;
      const double dy = poses[i].pose.position.y - robot_y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }

    const double s0 = arc_length[closest_idx];

    // 采样点（包含 closest_idx 处作为 k=0）
    struct Sample {
      double x = 0.0;
      double y = 0.0;
      double s = 0.0;
      double t = 0.0;
      bool at_end = false;  // 该采样点被夹到路径末端
    };
    std::vector<Sample> samples;
    samples.reserve(horizon_ + 1);

    for (int k = 0; k <= horizon_; ++k) {
      double target_s = s0 + static_cast<double>(k) * dt_ * ref_speed_;
      bool at_end = false;
      if (target_s >= total_length) {
        target_s = total_length;
        at_end = true;
        if (reached_end != nullptr) {
          *reached_end = true;
        }
      }

      // 找到 target_s 所在的线段
      int seg_idx = 0;
      for (int i = 0; i < n - 1; ++i) {
        if (arc_length[i] <= target_s && arc_length[i + 1] >= target_s) {
          seg_idx = i;
          break;
        }
      }

      const double seg_len = arc_length[seg_idx + 1] - arc_length[seg_idx];
      double ratio = 0.0;
      if (seg_len > 1e-6) {
        ratio = (target_s - arc_length[seg_idx]) / seg_len;
        ratio = std::clamp(ratio, 0.0, 1.0);
      }

      Sample smp;
      smp.x = poses[seg_idx].pose.position.x +
              ratio * (poses[seg_idx + 1].pose.position.x - poses[seg_idx].pose.position.x);
      smp.y = poses[seg_idx].pose.position.y +
              ratio * (poses[seg_idx + 1].pose.position.y - poses[seg_idx].pose.position.y);
      smp.s = target_s;
      smp.t = static_cast<double>(k) * dt_;
      smp.at_end = at_end;
      // 路径耗尽：后续采样点统一使用路径最后一个点的位姿，
      // 避免插值误差导致终点附近坐标拖尾
      if (at_end) {
        smp.x = poses[n - 1].pose.position.x;
        smp.y = poses[n - 1].pose.position.y;
      }
      samples.push_back(smp);
    }

    // 计算航向角
    std::vector<double> yaws(samples.size(), 0.0);
    for (size_t i = 0; i < samples.size(); ++i) {
      if (i + 1 < samples.size()) {
        const double dx = samples[i + 1].x - samples[i].x;
        const double dy = samples[i + 1].y - samples[i].y;
        yaws[i] = std::atan2(dy, dx);
      } else if (i > 0) {
        yaws[i] = yaws[i - 1];
      }
    }

    // 计算曲率并调整速度
    for (size_t i = 0; i < samples.size(); ++i) {
      double curvature = 0.0;
      if (samples.size() >= 3) {
        size_t i_prev = (i == 0) ? 0 : i - 1;
        size_t i_next = (i + 1 >= samples.size()) ? samples.size() - 1 : i + 1;
        curvature = computeCurvature(samples[i_prev].x, samples[i_prev].y,
                                     samples[i].x, samples[i].y,
                                     samples[i_next].x, samples[i_next].y);
      }

      // 弯道减速：曲率越大速度越低
      double speed = ref_speed_ / (1.0 + curvature_weight_ * curvature);
      if (speed > max_speed_) {
        speed = max_speed_;
      }
      if (speed < min_speed_) {
        speed = min_speed_;
      }
      // 路径耗尽：被夹到路径末端的采样点参考速度置 0，
      // 让求解器在接近终点时自然减速，避免 guidance 退化为
      // 同一坐标点但仍要求非零速度导致的振荡/不收敛
      if (samples[i].at_end) {
        speed = 0.0;
      }

      GuidancePoint gp;
      gp.x = samples[i].x;
      gp.y = samples[i].y;
      gp.yaw = yaws[i];
      gp.speed = speed;
      gp.t = samples[i].t;
      gp.s = samples[i].s;
      // d 为机器人当前位置到该点处路径切线的有符号横向距离
      gp.d = signedLateralDistance(robot_x, robot_y, samples[i].x, samples[i].y, yaws[i]);

      guidance.push_back(gp);
    }

    return guidance;
  }

 private:
  /**
   * @brief 用三点近似曲率（Menger 曲率）。
   */
  static double computeCurvature(double ax, double ay,
                                 double bx, double by,
                                 double cx, double cy) {
    const double ab_x = bx - ax;
    const double ab_y = by - ay;
    const double bc_x = cx - bx;
    const double bc_y = cy - by;
    const double ac_x = cx - ax;
    const double ac_y = cy - ay;

    const double a_len = std::hypot(bc_x, bc_y);
    const double b_len = std::hypot(ac_x, ac_y);
    const double c_len = std::hypot(ab_x, ab_y);

    if (a_len < 1e-6 || b_len < 1e-6 || c_len < 1e-6) {
      return 0.0;
    }

    // 三角形面积的两倍 = |AB x AC|
    const double cross = std::abs(ab_x * ac_y - ab_y * ac_x);
    if (cross < 1e-12) {
      return 0.0;
    }

    // 曲率 = 4 * 面积 / (a * b * c)
    const double area2 = cross;
    return area2 / (a_len * b_len * c_len);
  }

  /**
   * @brief 计算点 (px,py) 到过 (qx,qy) 方向为 yaw 的直线的有符号距离。
   */
  static double signedLateralDistance(double px, double py,
                                      double qx, double qy,
                                      double yaw) {
    const double dx = px - qx;
    const double dy = py - qy;
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);
    // 左侧为正
    return -dx * sin_yaw + dy * cos_yaw;
  }

  int horizon_ = 20;
  double dt_ = 0.1;
  double ref_speed_ = 1.0;
  double curvature_weight_ = 0.5;
  double min_speed_ = 0.0;
  double max_speed_ = 1.0;
};

}  // namespace al_ilqr_ros
