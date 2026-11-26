#pragma once
/**
 * local_planner.h
 * 局部路径规划: 预生成候选路径集 + 碰撞检测 + 评分
 *
 * 参考: autonomy_stack_go2 localPlanner.cpp
 * - 订阅 /terrain_map + /state_estimation + /nav_goal
 * - 发布 /local_path (nav_msgs/Path) + /cmd_vel
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace nav_algo {

class LocalPlanner {
public:
    LocalPlanner() = default;
    ~LocalPlanner() = default;

    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void terrainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void planTimerCallback(const ros::TimerEvent&);

    ros::Subscriber terrain_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher cmd_pub_;
    ros::Timer plan_timer_;

    // 参数
    double path_length_ = 4.0;
    int n_paths_ = 21;
    double max_angle_deg_ = 30.0;
};

}  // namespace nav_algo
