#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace al_ilqr_ros {

/**
 * @brief 将 sensor_msgs::LaserScan 转换为 AL-iLQR 可用的圆形障碍物列表。
 *
 * 处理流程：
 *   1. 将 scan 极坐标转为机器人坐标系下的笛卡尔坐标；
 *   2. 利用机器人位姿转换到 odom 坐标系；
 *   3. 对相邻点进行简单聚类；
 *   4. 将空间跨度过大的簇（如整面墙）拆分为多个小子簇，
 *      避免外接圆圆心在墙上但圆体侵入走廊中央、把可通行区堵死；
 *   5. 对每个类计算质心与外接圆半径（可附加安全半径 inflation）；
 *   6. 过滤过远或过小的障碍物。
 */
class ObstacleAdapter {
 public:
  struct CircleObstacle {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
  };

  ObstacleAdapter() = default;

  /**
   * @brief 配置聚类与过滤参数。
   *
   * @param clustering_distance 相邻 scan 点被视为同一类的最大距离 (m)
   * @param min_radius          障碍物最小半径，小于该值的类将被过滤 (m)
   * @param max_range           只处理 range 小于该值的点 (m)
   * @param max_radius          障碍物最大半径，大于该值的类将被过滤 (m)，
   *                            用于剔除把整面墙聚成一个巨大圆的异常簇；
   *                            <= 0 表示不做上限过滤
   */
  void configure(double clustering_distance, double min_radius, double max_range,
                 double max_radius = 0.0) {
    clustering_distance_ = (clustering_distance > 1e-6) ? clustering_distance : 1e-6;
    min_radius_ = std::max(0.0, min_radius);
    max_range_ = (max_range > 0.0) ? max_range : std::numeric_limits<double>::infinity();
    max_radius_ = max_radius;
  }

  /**
   * @brief 设置安全膨胀半径，最终障碍物半径 = 外接圆半径 + safety_radius。
   */
  void setSafetyRadius(double safety_radius) {
    safety_radius_ = std::max(0.0, safety_radius);
  }

  /**
   * @brief 设置单簇最大外接圆半径（不含膨胀）。
   *
   * 聚类后空间跨度超过 2*max_cluster_radius 的簇（典型如整段墙壁）
   * 会被拆分为多个小子簇。若不拆分，墙壁的外接圆圆心落在墙面上、
   * 半径可达米级，圆体会覆盖走廊中央，导致求解器认为“任何前进都
   * 违约”而收敛到原地不动的零违约局部最优（真机现象：cvg=1、
   * viol=0、a≈0）。<= 0 表示不拆分。
   */
  void setMaxClusterRadius(double max_cluster_radius) {
    max_cluster_radius_ = max_cluster_radius;
  }

  /**
   * @brief 从 LaserScan 提取 odom 坐标系下的圆形障碍物。
   *
   * 假设 scan 数据位于机器人 base 坐标系（或已通过 TF 与 base 对齐）。
   *
   * @param scan       输入激光雷达数据
   * @param robot_x    机器人在 odom 下的 x 坐标
   * @param robot_y    机器人在 odom 下的 y 坐标
   * @param robot_yaw  机器人在 odom 下的航向角 (rad)
   * @return std::vector<CircleObstacle> 检测到的障碍物
   */
  std::vector<CircleObstacle> fromLaserScan(const sensor_msgs::LaserScan& scan,
                                            double robot_x,
                                            double robot_y,
                                            double robot_yaw) {
    std::vector<CircleObstacle> obstacles;

    if (scan.ranges.empty()) {
      return obstacles;
    }

    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);

    // 第一步：将有效 scan 点转换到 odom 坐标系
    struct Point {
      double x = 0.0;
      double y = 0.0;
    };
    std::vector<Point> valid_points;
    valid_points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r <= scan.range_min || r >= max_range_) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);

      // base -> odom 的 2D 刚体变换
      Point p;
      p.x = robot_x + lx * cos_yaw - ly * sin_yaw;
      p.y = robot_y + lx * sin_yaw + ly * cos_yaw;
      valid_points.push_back(p);
    }

    if (valid_points.empty()) {
      return obstacles;
    }

    // 第二步：相邻点简单聚类
    std::vector<std::vector<Point>> clusters;
    clusters.emplace_back();
    clusters.back().push_back(valid_points.front());

    for (size_t i = 1; i < valid_points.size(); ++i) {
      const auto& prev = valid_points[i - 1];
      const auto& curr = valid_points[i];
      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      const double dist = std::hypot(dx, dy);

      if (dist < clustering_distance_) {
        clusters.back().push_back(curr);
      } else {
        clusters.emplace_back();
        clusters.back().push_back(curr);
      }
    }

    // 第三步：大簇拆分——簇内任意点到子簇起点的距离超过
    // 2*max_cluster_radius_（直径）时开启新子簇，保证每个子簇的
    // 外接圆半径大致 <= max_cluster_radius_。墙壁由此变为贴着墙面的
    // 一串小圆，走廊中央保持可通行。
    if (max_cluster_radius_ > 1e-6) {
      const double max_span = 2.0 * max_cluster_radius_;
      std::vector<std::vector<Point>> split_clusters;
      split_clusters.reserve(clusters.size());
      for (const auto& cluster : clusters) {
        size_t start = 0;
        for (size_t i = 1; i < cluster.size(); ++i) {
          const double dx = cluster[i].x - cluster[start].x;
          const double dy = cluster[i].y - cluster[start].y;
          if (std::hypot(dx, dy) > max_span) {
            split_clusters.push_back(std::vector<Point>(
                cluster.begin() + static_cast<std::ptrdiff_t>(start),
                cluster.begin() + static_cast<std::ptrdiff_t>(i)));
            start = i;
          }
        }
        split_clusters.push_back(std::vector<Point>(
            cluster.begin() + static_cast<std::ptrdiff_t>(start), cluster.end()));
      }
      clusters.swap(split_clusters);
    }

    // 第四步：对每个类计算质心与外接圆半径
    for (const auto& cluster : clusters) {
      if (cluster.empty()) {
        continue;
      }

      // 质心
      double cx = 0.0;
      double cy = 0.0;
      for (const auto& p : cluster) {
        cx += p.x;
        cy += p.y;
      }
      cx /= static_cast<double>(cluster.size());
      cy /= static_cast<double>(cluster.size());

      // 外接圆半径：类内点到质心的最大距离
      double max_r = 0.0;
      for (const auto& p : cluster) {
        const double r = std::hypot(p.x - cx, p.y - cy);
        if (r > max_r) {
          max_r = r;
        }
      }

      // 加上安全膨胀
      double final_radius = max_r + safety_radius_;

      // 过滤过小的障碍物（噪声）与过大的异常簇（整面墙/地面反射）
      if (final_radius < min_radius_) {
        continue;
      }
      if (max_radius_ > 0.0 && final_radius > max_radius_) {
        continue;
      }

      CircleObstacle obs;
      obs.x = cx;
      obs.y = cy;
      obs.radius = final_radius;
      obstacles.push_back(obs);
    }

    return obstacles;
  }

 private:
  double clustering_distance_ = 0.3;
  double min_radius_ = 0.1;
  double max_range_ = 5.0;
  double max_radius_ = 0.0;  // <= 0 表示不限制
  double max_cluster_radius_ = 0.0;  // 单簇最大外接圆半径，<= 0 表示不拆分
  double safety_radius_ = 0.0;
};

}  // namespace al_ilqr_ros
