#pragma once
/**
 * terrain_analysis.h
 * 地形分析: 从 registered_scan 提取地面 + 障碍物
 *
 * 参考: autonomy_stack_go2 terrainAnalysis.cpp
 * - 分位数地面估计 (planar voxel grid)
 * - 障碍物 = 高于地面 obstacle_height_thre 的点
 * - 发布 /terrain_map (sensor_msgs/PointCloud2, 带 label 字段)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>

namespace nav_algo {

class TerrainAnalysis {
public:
    TerrainAnalysis() = default;
    ~TerrainAnalysis() = default;

    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    ros::Subscriber cloud_sub_;
    ros::Publisher terrain_pub_;

    // 参数
    double planar_voxel_size_ = 1.0;
    double ground_quantile_ = 0.05;
    double obstacle_height_thre_ = 0.2;
};

}  // namespace nav_algo
