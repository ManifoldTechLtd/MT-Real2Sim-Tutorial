/**
 * path_follower_node.cpp
 * Pure Pursuit 路径跟踪节点 — 骨架 (Phase 2 实现)
 *
 * 订阅: /local_path, /state_estimation
 * 发布: /cmd_vel (geometry_msgs/Twist)
 */

#include <ros/ros.h>
#include "nav_algo/path_follower.h"

namespace nav_algo {

void PathFollower::init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param("lookahead_dist", lookahead_dist_, 1.0);
    pnh.param("max_v", max_v_, 3.0);
    pnh.param("max_w", max_w_, 3.0);
    pnh.param("goal_tolerance", goal_tolerance_, 0.3);

    path_sub_ = nh.subscribe("/local_path", 5,
                             &PathFollower::pathCallback, this);
    odom_sub_ = nh.subscribe("/state_estimation", 10,
                             &PathFollower::odomCallback, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    control_timer_ = nh.createTimer(ros::Duration(0.05),  // 20Hz
                                    &PathFollower::controlTimerCallback, this);

    ROS_INFO("[PathFollower] Initialized: lookahead=%.1f, max_v=%.1f, max_w=%.1f",
             lookahead_dist_, max_v_, max_w_);
}

void PathFollower::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // TODO Phase 2: 缓存路径
}

void PathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // TODO Phase 2: 更新机器人位姿
}

void PathFollower::controlTimerCallback(const ros::TimerEvent&) {
    // TODO Phase 2: Pure Pursuit 计算 cmd_vel
}

}  // namespace nav_algo

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    nav_algo::PathFollower pf;
    pf.init(nh, pnh);

    ros::spin();
    return 0;
}
