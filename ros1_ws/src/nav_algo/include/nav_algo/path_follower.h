#pragma once
/**
 * path_follower.h
 * Pure Pursuit 全向路径跟踪
 *
 * 参考: autonomy_stack_go2 pathFollower.cpp
 * - 订阅 /local_path + /state_estimation
 * - 发布 /cmd_vel (geometry_msgs/Twist)
 * - 倾斜安全机制 (IMU feedback)
 * - 方向切换防抖
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace nav_algo {

class PathFollower {
public:
    PathFollower() = default;
    ~PathFollower() = default;

    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);

    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;

    // 参数
    double lookahead_dist_ = 1.0;
    double max_v_ = 3.0;
    double max_w_ = 3.0;
    double goal_tolerance_ = 0.3;
};

}  // namespace nav_algo
