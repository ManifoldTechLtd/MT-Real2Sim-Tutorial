#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "al_ilqr_ros/bicycle_to_diff.hpp"
#include "al_ilqr_ros/obstacle_adapter.hpp"
#include "al_ilqr_ros/path_converter.hpp"
#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/guidance_tracker.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "core/types.hpp"

namespace al_ilqr_ros {

/**
 * @brief AL-iLQR ROS 规划节点。
 *
 * 订阅里程计、全局路径和激光雷达，周期性地调用 GuidanceTracker 求解最优控制，
 * 并将第一步控制量映射为差速命令发布。
 */
class AlIlqrRosNode {
 public:
  explicit AlIlqrRosNode(ros::NodeHandle nh) : nh_(nh), tf_listener_(tf_buffer_, nh_, true) {
    loadParameters();
    setupVehicle();
    setupAdapters();
    setupCommunications();
  }

  void run() {
    ros::spin();
  }

 private:
  /**
   * @brief 从参数服务器加载所有配置。
   */
  void loadParameters() {
    // 默认参数
    odom_topic_ = "/odin1/odometry";
    path_topic_ = "/map_planner/global_path";
    scan_topic_ = "/scan";
    cmd_vel_topic_ = "/cmd_vel";
    base_frame_ = "odin1_base_link";
    odom_frame_ = "odom";

    nh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    nh_.param<std::string>("path_topic", path_topic_, path_topic_);
    nh_.param<std::string>("scan_topic", scan_topic_, scan_topic_);
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    nh_.param<std::string>("base_frame", base_frame_, base_frame_);
    nh_.param<std::string>("odom_frame", odom_frame_, odom_frame_);

    // 车辆参数
    nh_.param("vehicle/wheelbase", vehicle_config_.model.wheelbase, 0.4);
    nh_.param("vehicle/max_speed", vehicle_config_.model.max_speed, 1.0);
    nh_.param("vehicle/min_speed", vehicle_config_.model.min_speed, -0.3);
    nh_.param("vehicle/max_acceleration", vehicle_config_.model.max_acceleration, 1.5);
    nh_.param("vehicle/min_acceleration", vehicle_config_.model.min_acceleration, -2.0);
    nh_.param("vehicle/max_steering", vehicle_config_.model.max_steering, 0.6);
    nh_.param("vehicle/min_steering", vehicle_config_.model.min_steering, -0.6);

    nh_.param("vehicle/length", vehicle_config_.body.length, 0.6);
    nh_.param("vehicle/width", vehicle_config_.body.width, 0.4);
    nh_.param("vehicle/rear_axle_to_rear", vehicle_config_.body.rear_axle_to_rear, 0.15);
    nh_.param("vehicle/collision_circle_count", vehicle_config_.body.collision_circle_count, 1);

    // Solver 参数
    nh_.param("solver/horizon", horizon_, 20);
    nh_.param("solver/dt", dt_, 0.1);
    nh_.param("solver/max_outer_iterations", tracker_config_.max_outer_iterations, 4);
    nh_.param("solver/max_inner_iterations", tracker_config_.max_inner_iterations, 15);
    nh_.param("solver/initial_penalty", tracker_config_.initial_penalty, 20.0);
    nh_.param("solver/penalty_scaling", tracker_config_.penalty_scaling, 5.0);
    nh_.param("solver/constraint_tolerance", tracker_config_.constraint_tolerance, 0.1);

    // 跟踪权重
    nh_.param("weights/position_weight", tracker_config_.position_weight, 5.0);
    nh_.param("weights/heading_weight", tracker_config_.heading_weight, 2.0);
    nh_.param("weights/speed_weight", tracker_config_.speed_weight, 1.0);
    nh_.param("weights/terminal_position_weight", tracker_config_.terminal_position_weight, 8.0);
    nh_.param("weights/terminal_heading_weight", tracker_config_.terminal_heading_weight, 4.0);
    nh_.param("weights/terminal_speed_weight", tracker_config_.terminal_speed_weight, 2.0);
    nh_.param("weights/accel_reg_weight", tracker_config_.accel_reg_weight, 0.2);
    nh_.param("weights/steering_reg_weight", tracker_config_.steering_reg_weight, 0.4);

    // 规划频率与参考速度
    nh_.param("planning/rate", planning_rate_, 10.0);
    nh_.param("planning/ref_speed", ref_speed_, vehicle_config_.model.max_speed);
    nh_.param("planning/guidance_lookahead", guidance_lookahead_, horizon_);

    // 道路边界（当前 GuidanceTracker 使用以 x 轴为参考线的上下边界）
    nh_.param("road/lower_bound", road_lower_bound_, -5.0);
    nh_.param("road/upper_bound", road_upper_bound_, 5.0);

    // 障碍物参数
    double clustering_distance = 0.3;
    double min_obstacle_radius = 0.1;
    double scan_max_range = 5.0;
    double safety_radius = 0.35;
    nh_.param("obstacle/clustering_distance", clustering_distance, 0.3);
    nh_.param("obstacle/min_obstacle_radius", min_obstacle_radius, 0.1);
    nh_.param("obstacle/scan_max_range", scan_max_range, 5.0);
    nh_.param("obstacle/safety_radius", safety_radius, 0.35);
    obstacle_adapter_.configure(clustering_distance, min_obstacle_radius, scan_max_range);
    obstacle_adapter_.setSafetyRadius(safety_radius);

    // 路径转换参数
    double curvature_weight = 0.5;
    double path_min_speed = vehicle_config_.model.min_speed;
    double path_max_speed = vehicle_config_.model.max_speed;
    nh_.param("path/curvature_weight", curvature_weight, 0.5);
    nh_.param("path/min_speed", path_min_speed, vehicle_config_.model.min_speed);
    nh_.param("path/max_speed", path_max_speed, vehicle_config_.model.max_speed);
    path_converter_.configure(guidance_lookahead_, dt_, ref_speed_);
    path_converter_.configureSpeedScaling(curvature_weight, path_min_speed, path_max_speed);

    // 控制映射参数
    nh_.param("control_mapping/virtual_wheelbase", virtual_wheelbase_, 0.4);
    nh_.param("control_mapping/max_linear_vel", max_linear_vel_, 1.0);
    nh_.param("control_mapping/max_angular_vel", max_angular_vel_, 1.5);

    ROS_INFO("AL-iLQR ROS node parameters loaded:");
    ROS_INFO("  topics: odom=%s, path=%s, scan=%s, cmd=%s",
             odom_topic_.c_str(), path_topic_.c_str(),
             scan_topic_.c_str(), cmd_vel_topic_.c_str());
    ROS_INFO("  frames: base=%s, odom=%s", base_frame_.c_str(), odom_frame_.c_str());
    ROS_INFO("  vehicle: wheelbase=%.3f, max_speed=%.3f",
             vehicle_config_.model.wheelbase, vehicle_config_.model.max_speed);
    ROS_INFO("  solver: horizon=%d, dt=%.3f, rate=%.1f Hz", horizon_, dt_, planning_rate_);
  }

  /**
   * @brief 根据参数构造车辆几何与跟踪器。
   */
  void setupVehicle() {
    vehicle_geometry_ =
        std::make_unique<my_al_ilqr::VehicleCircleApproximation>(vehicle_config_.body);
    tracker_ = std::make_unique<my_al_ilqr::GuidanceTracker>(
        vehicle_config_, road_lower_bound_, road_upper_bound_, *vehicle_geometry_);
  }

  /**
   * @brief 初始化适配器（已在 loadParameters 中完成配置）。
   */
  void setupAdapters() {
    // 配置在 loadParameters 中已设置
  }

  /**
   * @brief 初始化 ROS 订阅、发布与定时器。
   */
  void setupCommunications() {
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &AlIlqrRosNode::odometryCallback, this);
    path_sub_ = nh_.subscribe(path_topic_, 1, &AlIlqrRosNode::pathCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 1, &AlIlqrRosNode::scanCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("/al_ilqr/optimized_path", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/al_ilqr/status", 1);

    const double period = (planning_rate_ > 1e-6) ? (1.0 / planning_rate_) : 0.1;
    planning_timer_ = nh_.createTimer(ros::Duration(period), &AlIlqrRosNode::planningLoop, this);
  }

  /**
   * @brief 里程计回调。
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  /**
   * @brief 全局路径回调。
   */
  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_path_ = *msg;
    has_path_ = true;
  }

  /**
   * @brief 激光雷达回调。
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_scan_ = *msg;
    has_scan_ = true;
  }

  /**
   * @brief 主规划循环。
   */
  void planningLoop(const ros::TimerEvent& /*event*/) {
    // 复制传感器数据，避免长时间持有锁
    nav_msgs::Odometry odom;
    nav_msgs::Path path;
    sensor_msgs::LaserScan scan;
    bool has_odom, has_path, has_scan;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom = latest_odom_;
      path = latest_path_;
      scan = latest_scan_;
      has_odom = has_odom_;
      has_path = has_path_;
      has_scan = has_scan_;
    }

    if (!has_odom) {
      publishStatus("WAITING_FOR_ODOM");
      publishZeroVelocity();
      return;
    }

    // 获取当前状态 [x, y, yaw, speed]
    double x = 0.0, y = 0.0, yaw = 0.0, speed = 0.0;
    if (!getRobotPose(x, y, yaw)) {
      // TF 失败时回退到里程计
      x = odom.pose.pose.position.x;
      y = odom.pose.pose.position.y;
      yaw = tf2::getYaw(odom.pose.pose.orientation);
    }
    speed = odom.twist.twist.linear.x;

    if (!has_path) {
      publishStatus("WAITING_FOR_PATH");
      publishZeroVelocity();
      return;
    }

    // 提取 guidance points
    auto guidance = path_converter_.extractGuidance(path, x, y, yaw, speed);
    if (guidance.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: extracted guidance too short, stopping.");
      publishStatus("GUIDANCE_TOO_SHORT");
      publishZeroVelocity();
      return;
    }

    // 构造障碍物（用于状态监控与未来扩展）
    if (has_scan) {
      auto obstacles = obstacle_adapter_.fromLaserScan(scan, x, y, yaw);
      // GuidanceTracker 当前版本不接收障碍物，因此仅记录数量。
      // 若后续 tracker 接口扩展，可在此处传入。
      ROS_DEBUG("AL-iLQR: detected %zu obstacles from scan.", obstacles.size());
    }

    // 准备初始状态
    my_al_ilqr::Vector initial_state(4);
    initial_state << x, y, yaw, speed;

    // 准备 warm start 控制序列
    std::vector<my_al_ilqr::Vector> initial_controls(horizon_, my_al_ilqr::Vector::Zero(2));
    if (static_cast<int>(warm_start_controls_.size()) >= horizon_) {
      // 时移：上一帧的 k+1 步作为当前帧的 k 步初始猜测
      for (int k = 0; k < horizon_; ++k) {
        if (k + 1 < static_cast<int>(warm_start_controls_.size())) {
          initial_controls[k] = warm_start_controls_[k + 1];
        } else {
          initial_controls[k] = warm_start_controls_.back();
        }
      }
    }

    // 调用 GuidanceTracker 求解
    auto result = tracker_->Track(initial_state, guidance, initial_controls,
                                  horizon_, dt_, tracker_config_);

    if (result.controls.empty() || !result.converged) {
      ROS_WARN_THROTTLE(1.0,
                        "AL-iLQR: solver did not converge (converged=%d, controls=%zu), stopping.",
                        static_cast<int>(result.converged), result.controls.size());
      publishStatus("SOLVER_FAILED");
      publishZeroVelocity();
      warm_start_controls_.clear();
      return;
    }

    // 保存控制序列作为下一帧 warm start
    warm_start_controls_ = result.controls;

    // 取第一步控制量 [accel, steering]
    const double accel = result.controls[0](0);
    const double steering = result.controls[0](1);

    // 映射为差速控制
    auto cmd = bicycleToDiff(accel, steering, speed, dt_, virtual_wheelbase_,
                             max_linear_vel_, max_angular_vel_);

    // 发布控制指令
    publishCmdVel(cmd.v_linear, cmd.omega);

    // 发布可视化轨迹
    publishOptimizedPath(guidance, result.states);

    // 发布状态
    char status_msg[256];
    std::snprintf(status_msg, sizeof(status_msg),
                  "OK | converge=%d | iter=(%d outer, %d inner) | solve_time=%.2f ms | "
                  "violation=%.3f | accel=%.3f | steer=%.3f | v=%.3f | omega=%.3f",
                  static_cast<int>(result.converged),
                  result.outer_iterations,
                  result.total_inner_iterations,
                  result.solve_time_ms,
                  result.max_violation,
                  accel,
                  steering,
                  cmd.v_linear,
                  cmd.omega);
    publishStatus(status_msg);

    ROS_DEBUG("AL-iLQR: %s", status_msg);
  }

  /**
   * @brief 通过 TF 获取机器人在 odom 下的位姿。
   * @return true 表示成功
   */
  bool getRobotPose(double& x, double& y, double& yaw) {
    try {
      geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: TF lookup failed: %s", ex.what());
      return false;
    }
  }

  /**
   * @brief 发布差速速度指令。
   */
  void publishCmdVel(double v, double omega) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = omega;
    cmd_vel_pub_.publish(cmd);
  }

  /**
   * @brief 发布零速度，作为安全保护。
   */
  void publishZeroVelocity() {
    publishCmdVel(0.0, 0.0);
  }

  /**
   * @brief 发布优化后的轨迹用于可视化。
   */
  void publishOptimizedPath(const std::vector<my_al_ilqr::GuidancePoint>& guidance,
                            const std::vector<my_al_ilqr::Vector>& states) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = odom_frame_;

    // 优先使用 solver 返回的状态序列；若为空则退化为 guidance
    const bool use_states = (!states.empty() && states.front().size() >= 2);
    const size_t count = use_states ? states.size() : guidance.size();

    path.poses.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      if (use_states) {
        pose.pose.position.x = states[i](0);
        pose.pose.position.y = states[i](1);
        pose.pose.position.z = 0.0;
        const double yaw = (states[i].size() >= 3) ? states[i](2) : 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
      } else {
        pose.pose.position.x = guidance[i].x;
        pose.pose.position.y = guidance[i].y;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, guidance[i].yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
      }
      path.poses.push_back(pose);
    }

    optimized_path_pub_.publish(path);
  }

  /**
   * @brief 发布状态字符串。
   */
  void publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
  }

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher optimized_path_pub_;
  ros::Publisher status_pub_;
  ros::Timer planning_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 传感器数据缓存
  std::mutex state_mutex_;
  nav_msgs::Odometry latest_odom_;
  nav_msgs::Path latest_path_;
  sensor_msgs::LaserScan latest_scan_;
  bool has_odom_ = false;
  bool has_path_ = false;
  bool has_scan_ = false;

  // 参数
  std::string odom_topic_;
  std::string path_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  double planning_rate_ = 10.0;
  int guidance_lookahead_ = 20;

  // 车辆与求解器
  my_al_ilqr::VehicleBicycleConfig vehicle_config_;
  my_al_ilqr::GuidanceTrackerConfig tracker_config_;
  std::unique_ptr<my_al_ilqr::VehicleCircleApproximation> vehicle_geometry_;
  std::unique_ptr<my_al_ilqr::GuidanceTracker> tracker_;

  // 适配器
  PathConverter path_converter_;
  ObstacleAdapter obstacle_adapter_;

  // 求解参数
  int horizon_ = 20;
  double dt_ = 0.1;
  double ref_speed_ = 1.0;
  double road_lower_bound_ = -5.0;
  double road_upper_bound_ = 5.0;

  // 控制映射
  double virtual_wheelbase_ = 0.4;
  double max_linear_vel_ = 1.0;
  double max_angular_vel_ = 1.5;

  // Warm start
  std::vector<my_al_ilqr::Vector> warm_start_controls_;
};

}  // namespace al_ilqr_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "al_ilqr_planner");
  ros::NodeHandle nh("~");
  ROS_INFO("AL-iLQR Planner node started.");

  try {
    al_ilqr_ros::AlIlqrRosNode node(nh);
    node.run();
  } catch (const std::exception& e) {
    ROS_FATAL("AL-iLQR node exception: %s", e.what());
    return 1;
  }

  return 0;
}
