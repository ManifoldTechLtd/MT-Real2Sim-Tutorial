/**
 * local_planner_node.cpp
 * 局部路径规划节点 — 骨架 (Phase 2 实现)
 *
 * 订阅: /terrain_map, /state_estimation, /nav_goal
 * 发布: /local_path (nav_msgs/Path)
 */

#include <ros/ros.h>
#include "nav_algo/local_planner.h"

namespace nav_algo {

void LocalPlanner::init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("path_length", path_length_, 4.0);
    pnh.param("n_paths", n_paths_, 21);
    pnh.param("max_angle_deg", max_angle_deg_, 30.0);

    terrain_sub_ = nh.subscribe("/terrain_map", 5,
                                &LocalPlanner::terrainCallback, this);
    odom_sub_ = nh.subscribe("/state_estimation", 10,
                             &LocalPlanner::odomCallback, this);
    goal_sub_ = nh.subscribe("/nav_goal", 1,
                             &LocalPlanner::goalCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("/local_path", 5);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    plan_timer_ = nh.createTimer(ros::Duration(0.1),
                                 &LocalPlanner::planTimerCallback, this);

    ROS_INFO("[LocalPlanner] Initialized: path_len=%.1f, n_paths=%d, max_angle=%.1f",
             path_length_, n_paths_, max_angle_deg_);
}

void LocalPlanner::terrainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // TODO Phase 2: 更新地形障碍物缓存
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // TODO Phase 2: 更新机器人位姿
}

void LocalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // TODO Phase 2: 设置导航目标
    ROS_INFO("[LocalPlanner] New goal: (%.2f, %.2f)",
             msg->pose.position.x, msg->pose.position.y);
}

void LocalPlanner::planTimerCallback(const ros::TimerEvent&) {
    // TODO Phase 2: 候选路径生成 + 评分 + 发布最优路径
}

}  // namespace nav_algo

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    nav_algo::LocalPlanner lp;
    lp.init(nh, pnh);

    ros::spin();
    return 0;
}
