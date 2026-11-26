/**
 * terrain_analysis_node.cpp
 * 地形分析节点 — 骨架 (Phase 2 实现)
 *
 * 订阅: /registered_scan (PointCloud2)
 * 发布: /terrain_map (PointCloud2, 带障碍物标记)
 */

#include <ros/ros.h>
#include "nav_algo/terrain_analysis.h"

namespace nav_algo {

void TerrainAnalysis::init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("planar_voxel_size", planar_voxel_size_, 1.0);
    pnh.param("ground_quantile", ground_quantile_, 0.05);
    pnh.param("obstacle_height_thre", obstacle_height_thre_, 0.2);

    cloud_sub_ = nh.subscribe("/registered_scan", 5,
                              &TerrainAnalysis::cloudCallback, this);
    terrain_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/terrain_map", 5);

    ROS_INFO("[TerrainAnalysis] Initialized: voxel=%.1f, quantile=%.2f, thre=%.2f",
             planar_voxel_size_, ground_quantile_, obstacle_height_thre_);
}

void TerrainAnalysis::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // TODO Phase 2: 分位数地面估计 + 障碍物检测
    // 暂时直接转发
    terrain_pub_.publish(*msg);
}

}  // namespace nav_algo

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_analysis_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    nav_algo::TerrainAnalysis ta;
    ta.init(nh, pnh);

    ros::spin();
    return 0;
}
