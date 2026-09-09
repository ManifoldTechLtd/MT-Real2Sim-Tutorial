#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
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
    // 默认参数（与 config/al_ilqr_go2.yaml、launch 中的话题保持一致）
    odom_topic_ = "/odin1/odometry";
    path_topic_ = "/initial_path";
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

    // 车辆参数合法性：非法值会让 KinematicBicycleModel / SpeedLimitConstraint /
    // VehicleCircleApproximation 在求解时抛异常，直接导致节点退出
    if (!(vehicle_config_.model.wheelbase > 1e-3)) {
      ROS_WARN("AL-iLQR: vehicle/wheelbase=%.4f invalid, reset to 0.4 m.",
               vehicle_config_.model.wheelbase);
      vehicle_config_.model.wheelbase = 0.4;
    }
    if (vehicle_config_.model.min_speed > vehicle_config_.model.max_speed) {
      ROS_WARN("AL-iLQR: vehicle/min_speed > max_speed, swapped.");
      std::swap(vehicle_config_.model.min_speed, vehicle_config_.model.max_speed);
    }
    if (vehicle_config_.model.min_acceleration > vehicle_config_.model.max_acceleration) {
      ROS_WARN("AL-iLQR: vehicle/min_acceleration > max_acceleration, swapped.");
      std::swap(vehicle_config_.model.min_acceleration,
                vehicle_config_.model.max_acceleration);
    }
    if (vehicle_config_.model.min_steering > vehicle_config_.model.max_steering) {
      ROS_WARN("AL-iLQR: vehicle/min_steering > max_steering, swapped.");
      std::swap(vehicle_config_.model.min_steering, vehicle_config_.model.max_steering);
    }
    // 转向角必须严格在 ±pi/2 内：JacobianControl 含 1/cos^2(delta)
    const double steer_limit = 0.5 * M_PI - 1e-2;
    vehicle_config_.model.max_steering =
        std::min(vehicle_config_.model.max_steering, steer_limit);
    vehicle_config_.model.min_steering =
        std::max(vehicle_config_.model.min_steering, -steer_limit);
    if (vehicle_config_.body.collision_circle_count < 1) {
      ROS_WARN("AL-iLQR: vehicle/collision_circle_count=%d invalid, reset to 1.",
               vehicle_config_.body.collision_circle_count);
      vehicle_config_.body.collision_circle_count = 1;
    }
    if (!(vehicle_config_.body.length > 0.0) || !(vehicle_config_.body.width > 0.0)) {
      ROS_WARN("AL-iLQR: vehicle/length or width invalid, reset to 0.6 x 0.4 m.");
      vehicle_config_.body.length = 0.6;
      vehicle_config_.body.width = 0.4;
    }
    // rear_axle_to_rear 必须在 [0, length) 内（VehicleCircleApproximation 硬要求）
    if (vehicle_config_.body.rear_axle_to_rear < 0.0 ||
        vehicle_config_.body.rear_axle_to_rear >= vehicle_config_.body.length) {
      const double fixed = 0.25 * vehicle_config_.body.length;
      ROS_WARN("AL-iLQR: vehicle/rear_axle_to_rear=%.3f out of [0, length=%.3f), reset to %.3f.",
               vehicle_config_.body.rear_axle_to_rear, vehicle_config_.body.length, fixed);
      vehicle_config_.body.rear_axle_to_rear = fixed;
    }

    // Solver 参数
    nh_.param("solver/horizon", horizon_, 20);
    nh_.param("solver/dt", dt_, 0.1);
    nh_.param("solver/max_outer_iterations", tracker_config_.max_outer_iterations, 4);
    nh_.param("solver/max_inner_iterations", tracker_config_.max_inner_iterations, 15);
    nh_.param("solver/initial_penalty", tracker_config_.initial_penalty, 20.0);
    nh_.param("solver/penalty_scaling", tracker_config_.penalty_scaling, 5.0);
    nh_.param("solver/constraint_tolerance", tracker_config_.constraint_tolerance, 0.1);
    // 内层 iLQR 代价收敛阈值（此前 yaml 有键但 C++ 未读取，求解器内部硬编码 1e-4）
    nh_.param("solver/convergence_threshold", tracker_config_.convergence_threshold, 1.0e-3);

    // 求导模式：analytical = 解析求导（闭式 Jacobian/Hessian），
    // numerical / finite_difference = 中心差分（仅用于对照与兜底）。
    // 该参数此前只写在 yaml 中、C++ 未读取，导致实际一直跑中心差分
    //（NX 上 ~124ms/次，与差分基准吻合）。
    std::string derivative_mode = "analytical";
    nh_.param<std::string>("solver/derivative_mode", derivative_mode, derivative_mode);
    std::string mode_key = derivative_mode;
    std::transform(mode_key.begin(), mode_key.end(), mode_key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode_key == "analytical" || mode_key == "analytic") {
      tracker_config_.use_analytical_derivatives = true;
    } else if (mode_key == "numerical" || mode_key == "finite_difference" ||
               mode_key == "finite-difference" || mode_key == "fd") {
      tracker_config_.use_analytical_derivatives = false;
    } else {
      ROS_WARN("AL-iLQR: unknown solver/derivative_mode='%s' (expect analytical|numerical), "
               "falling back to analytical.", derivative_mode.c_str());
      tracker_config_.use_analytical_derivatives = true;
    }
    derivative_mode_ = tracker_config_.use_analytical_derivatives ? "analytical" : "numerical";

    // 求解器参数合法性（非法值会让 OptimalControlProblem/ALILQRSolver 抛异常）
    if (horizon_ < 2) {
      ROS_WARN("AL-iLQR: solver/horizon=%d invalid, clamped to 2.", horizon_);
      horizon_ = 2;
    }
    if (!(dt_ > 1e-3)) {
      ROS_WARN("AL-iLQR: solver/dt=%.4f invalid, reset to 0.1 s.", dt_);
      dt_ = 0.1;
    }
    if (tracker_config_.max_outer_iterations < 1) {
      tracker_config_.max_outer_iterations = 1;
    }
    if (tracker_config_.max_inner_iterations < 1) {
      tracker_config_.max_inner_iterations = 1;
    }
    if (!(tracker_config_.initial_penalty > 0.0)) {
      tracker_config_.initial_penalty = 20.0;
    }
    if (!(tracker_config_.penalty_scaling > 1.0)) {
      tracker_config_.penalty_scaling = 5.0;
    }
    if (!(tracker_config_.constraint_tolerance > 0.0)) {
      tracker_config_.constraint_tolerance = 0.1;
    }
    if (!(tracker_config_.convergence_threshold > 0.0)) {
      tracker_config_.convergence_threshold = 1.0e-3;
    }

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
    // 道路边界约束开关：该约束是绝对坐标系 y 向走廊（非真车道线），
    // 真实部署默认禁用
    nh_.param("planning/enable_road_boundary", enable_road_boundary_, false);
    // 里程计数据超时（s）：超过该时长未收到新 odom 视为数据失效，
    // 输出零速并报 ODOM_TIMEOUT。<= 0 表示禁用该检查。
    nh_.param("planning/odom_timeout", odom_timeout_, 0.5);

    if (!(planning_rate_ > 1e-3)) {
      ROS_WARN("AL-iLQR: planning/rate=%.3f invalid, reset to 10 Hz.", planning_rate_);
      planning_rate_ = 10.0;
    }
    // guidance 采样窗口必须覆盖到 horizon（含终端点 index=horizon），
    // 否则终端参考点会被夹到窗口末尾，终端权重失效
    if (guidance_lookahead_ < horizon_) {
      ROS_WARN("AL-iLQR: planning/guidance_lookahead=%d < solver/horizon=%d, "
               "raised to horizon.", guidance_lookahead_, horizon_);
      guidance_lookahead_ = horizon_;
    }
    // 参考速度不得超过动力学速度上限，否则速度约束与跟踪代价长期冲突
    if (!std::isfinite(ref_speed_) || ref_speed_ < 0.0) {
      ref_speed_ = std::max(0.0, vehicle_config_.model.max_speed);
    }
    if (ref_speed_ > vehicle_config_.model.max_speed) {
      ROS_WARN("AL-iLQR: planning/ref_speed=%.3f > vehicle/max_speed=%.3f, clamped.",
               ref_speed_, vehicle_config_.model.max_speed);
      ref_speed_ = vehicle_config_.model.max_speed;
    }

    // 道路边界（当前 GuidanceTracker 使用以 x 轴为参考线的上下边界）
    nh_.param("road/lower_bound", road_lower_bound_, -5.0);
    nh_.param("road/upper_bound", road_upper_bound_, 5.0);

    // 动态障碍物参数（/scan，来自 pointcloud_to_laserscan 的前向 90° FOV 激光）
    // 注意：所有障碍物相关参数统一收敛到 dynamic_obstacles 段，
    //       旧的 obstacle 段（其中多数键从未被读取）已从 yaml 中移除。
    nh_.param("dynamic_obstacles/enable", enable_dynamic_obstacles_, false);
    nh_.param<std::string>("dynamic_obstacles/scan_topic", dynamic_scan_topic_, "/scan");
    nh_.param("dynamic_obstacles/inflate_radius", obstacle_inflate_radius_, 0.3);
    nh_.param("dynamic_obstacles/max_obstacles", max_obstacles_, 10);
    nh_.param("dynamic_obstacles/cluster_tolerance", cluster_tolerance_, 0.3);
    nh_.param("dynamic_obstacles/max_range", obstacle_max_range_, 5.0);
    nh_.param("dynamic_obstacles/min_radius", min_obstacle_radius_, 0.05);
    nh_.param("dynamic_obstacles/max_radius", max_obstacle_radius_, 2.0);
    nh_.param("dynamic_obstacles/timeout", dynamic_obstacles_timeout_, 1.0);
    // 单簇最大外接圆半径（不含膨胀）：超过该跨度的簇（典型如整段墙）
    // 会被拆分为多个小子簇，避免墙壁外接圆覆盖走廊中央堵死前进方向
    //（真机现象：求解收敛但 a≈0、机器人原地不动）。<=0 禁用拆分。
    nh_.param("dynamic_obstacles/max_cluster_radius", max_cluster_radius_, 0.4);
    if (max_obstacles_ < 0) {
      max_obstacles_ = 0;
    }
    if (!(cluster_tolerance_ > 1e-3)) {
      cluster_tolerance_ = 0.3;
    }
    if (obstacle_inflate_radius_ < 0.0) {
      obstacle_inflate_radius_ = 0.0;
    }
    dynamic_obstacle_adapter_.configure(cluster_tolerance_, min_obstacle_radius_,
                                       obstacle_max_range_, max_obstacle_radius_);
    dynamic_obstacle_adapter_.setSafetyRadius(obstacle_inflate_radius_);
    dynamic_obstacle_adapter_.setMaxClusterRadius(max_cluster_radius_);

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
    // 速度指令前视步数：cmd_vel 线速度取规划轨迹第 k 步的速度
    // states[k](3)，而非增量式 "odom 速度 + a*dt"。后者在静止起步时
    // 每周期最多爬升 max_accel*dt（≈ 0.1 m/s，落在 Go2 死区内），
    // 而机器人不动时 odom 速度恒为 0，形成自举死锁。取 0 退化回旧行为。
    nh_.param("control_mapping/speed_lookahead_steps", speed_lookahead_steps_, 3);
    if (speed_lookahead_steps_ < 0) {
      speed_lookahead_steps_ = 0;
    }
    if (speed_lookahead_steps_ > horizon_) {
      speed_lookahead_steps_ = horizon_;
    }
    // 停滞恢复：求解收敛但速度指令近零（且未接近路径末端）连续
    // stall_recovery_cycles 个周期后，清空 warm start 让下一周期回退
    // Pure-Pursuit 初值（含前向加速度），跳出“静止=零违约”局部最优。
    // stall_cmd_threshold <= 0 禁用该机制。
    nh_.param("planning/stall_cmd_threshold", stall_cmd_threshold_, 0.05);
    nh_.param("planning/stall_recovery_cycles", stall_recovery_cycles_, 3);
    if (stall_recovery_cycles_ < 1) {
      stall_recovery_cycles_ = 1;
    }

    // 到达判定参数：机器人与全局路径终点距离 < position_tolerance 时
    // 判定到达，发布 std_msgs/Empty 到 arrive_topic（默认 /neupan/arrive，
    // 与 Odin goal_state_machine 的订阅一致，用于触发到达后自动重规划）
    nh_.param("goal/position_tolerance", goal_position_tolerance_, 0.3);
    nh_.param("goal/publish_arrive", publish_arrive_, true);
    nh_.param<std::string>("goal/arrive_topic", arrive_topic_, "/neupan/arrive");
    // 朝向容差：仅当 goal/check_heading=true 时参与到达判定。
    // 默认关闭——全局路径终点的 orientation 由 map_planner 生成，
    // 通常不代表用户期望的停车朝向，强制对齐会导致原地转圈。
    nh_.param("goal/heading_tolerance", goal_heading_tolerance_, 0.2);
    nh_.param("goal/check_heading", goal_check_heading_, false);
    if (!(goal_position_tolerance_ > 0.0)) {
      goal_position_tolerance_ = 0.3;
    }
    if (!(goal_heading_tolerance_ > 0.0)) {
      goal_heading_tolerance_ = 0.2;
    }

    ROS_INFO("AL-iLQR ROS node parameters loaded:");
    ROS_INFO("  topics: odom=%s, path=%s, scan=%s, cmd=%s",
             odom_topic_.c_str(), path_topic_.c_str(),
             scan_topic_.c_str(), cmd_vel_topic_.c_str());
    ROS_INFO("  frames: base=%s, odom=%s (path frame_id is transformed to odom via TF)",
             base_frame_.c_str(), odom_frame_.c_str());
    ROS_INFO("  vehicle: wheelbase=%.3f, max_speed=%.3f",
             vehicle_config_.model.wheelbase, vehicle_config_.model.max_speed);
    ROS_INFO("  solver: horizon=%d, dt=%.3f, rate=%.1f Hz, derivative=%s, "
             "outer=%d, inner=%d, cost_tol=%.2e, cons_tol=%.3f",
             horizon_, dt_, planning_rate_, derivative_mode_.c_str(),
             tracker_config_.max_outer_iterations, tracker_config_.max_inner_iterations,
             tracker_config_.convergence_threshold, tracker_config_.constraint_tolerance);
    ROS_INFO("  planning: ref_speed=%.2f, guidance_lookahead=%d, odom_timeout=%.2f s",
             ref_speed_, guidance_lookahead_, odom_timeout_);
    ROS_INFO("  road_boundary: enable=%d (y-corridor [%.1f, %.1f] in odom frame, not a real lane)",
             static_cast<int>(enable_road_boundary_), road_lower_bound_, road_upper_bound_);
    ROS_INFO("  dynamic_obstacles: enable=%d, topic=%s, inflate=%.3f, max=%d, cluster_tol=%.3f, "
             "range=%.1f, radius=[%.2f, %.2f], max_cluster_radius=%.2f",
             static_cast<int>(enable_dynamic_obstacles_), dynamic_scan_topic_.c_str(),
             obstacle_inflate_radius_, max_obstacles_, cluster_tolerance_,
             obstacle_max_range_, min_obstacle_radius_, max_obstacle_radius_,
             max_cluster_radius_);
    ROS_INFO("  control_mapping: wheelbase=%.2f, max_v=%.2f, max_w=%.2f, "
             "speed_lookahead_steps=%d; stall: cmd_thresh=%.3f, cycles=%d",
             virtual_wheelbase_, max_linear_vel_, max_angular_vel_,
             speed_lookahead_steps_, stall_cmd_threshold_, stall_recovery_cycles_);
    ROS_INFO("  goal: pos_tol=%.2f m, heading_check=%d (tol=%.2f rad), publish_arrive=%d, topic=%s",
             goal_position_tolerance_, static_cast<int>(goal_check_heading_),
             goal_heading_tolerance_, static_cast<int>(publish_arrive_),
             arrive_topic_.c_str());
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

    // 动态障碍物：订阅 dynamic_scan_topic_（默认 /scan）用于 iLQR 约束。
    // 若与监控用 scan_topic_ 相同，则不重复订阅，改由 scanCallback 内直接
    // 调用 dynamicScanCallback，避免同一话题被订阅两次。
    if (enable_dynamic_obstacles_ && dynamic_scan_topic_ != scan_topic_) {
      dynamic_scan_sub_ = nh_.subscribe(dynamic_scan_topic_, 1,
                                        &AlIlqrRosNode::dynamicScanCallback, this);
    }

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("/al_ilqr/optimized_path", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/al_ilqr/status", 1);

    // 到达事件发布（std_msgs/Empty，与 goal_state_machine 的
    // arriveCallback(std_msgs::EmptyConstPtr) 订阅类型一致）
    if (publish_arrive_) {
      arrive_pub_ = nh_.advertise<std_msgs::Empty>(arrive_topic_, 1);
    }

    const double period = (planning_rate_ > 1e-6) ? (1.0 / planning_rate_) : 0.1;
    control_period_ = period;
    planning_timer_ = nh_.createTimer(ros::Duration(period), &AlIlqrRosNode::planningLoop, this);
  }

  /**
   * @brief 里程计回调。
   */
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 数值安全：里程计出现 NaN/inf 时直接丢弃，避免污染求解初值
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w) ||
        (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < 1e-6 ||
        !std::isfinite(msg->twist.twist.linear.x)) {
      ROS_WARN_THROTTLE(2.0, "AL-iLQR: dropped odometry with invalid (NaN/inf/zero-quat) data.");
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = *msg;
    // stamp 可能为 0（部分驱动不填），此时用接收时刻做新鲜度判断
    last_odom_time_ = (msg->header.stamp.isZero()) ? ros::Time::now() : msg->header.stamp;
    has_odom_ = true;
  }

  /**
   * @brief 全局路径回调。
   *
   * 只做缓存与基本校验；坐标系变换（path.header.frame_id -> odom）在
   * planningLoop 中用最新 TF 完成，保证 map->odom 漂移被实时补偿。
   */
  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // 数值安全：剔除含 NaN/inf 的路径点（map_planner 异常时出现过）
    nav_msgs::Path sanitized;
    sanitized.header = msg->header;
    sanitized.poses.reserve(msg->poses.size());
    size_t dropped = 0;
    for (const auto& ps : msg->poses) {
      if (std::isfinite(ps.pose.position.x) && std::isfinite(ps.pose.position.y) &&
          std::isfinite(ps.pose.position.z)) {
        sanitized.poses.push_back(ps);
      } else {
        ++dropped;
      }
    }
    if (dropped > 0) {
      ROS_WARN_THROTTLE(2.0, "AL-iLQR: dropped %zu non-finite path points.", dropped);
    }
    if (sanitized.poses.size() < 2) {
      ROS_WARN_THROTTLE(2.0, "AL-iLQR: received path with %zu valid poses (<2), ignored.",
                        sanitized.poses.size());
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_path_ = std::move(sanitized);
    has_path_ = true;
    // 新路径到来 -> 复位到达事件防重发标记，允许对新目标再次发布 arrive
    arrive_published_ = false;
  }

  /**
   * @brief 激光雷达回调（/scan，用于监控）。
   *
   * 仅提取最近障碍距离（O(n) 且无内存拷贝）用于状态诊断；
   * 完整的聚类只在动态避障回调中做一次，避免规划循环里重复聚类。
   * 若动态障碍话题与监控话题相同（默认均为 /scan），在此回调中
   * 同时触发动态障碍物处理，避免重复订阅同一话题。
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double min_range = std::numeric_limits<double>::infinity();
    for (const auto r : msg->ranges) {
      const double range = static_cast<double>(r);
      if (std::isfinite(range) && range > msg->range_min && range < min_range) {
        min_range = range;
      }
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      min_scan_range_ = min_range;
      has_scan_ = true;
    }

    if (enable_dynamic_obstacles_ && dynamic_scan_topic_ == scan_topic_) {
      dynamicScanCallback(msg);
    }
  }

  /**
   * @brief 动态障碍物激光回调（默认 /scan，来自 pointcloud_to_laserscan，
   *        前向 90° FOV，用于动态障碍物约束）。
   *
   * 将 LaserScan 转为 odom 坐标系下的笛卡尔点，使用简单欧式聚类
   * （相邻点距离 < cluster_tolerance_ 归为一簇），每簇计算质心和
   * 外接圆半径（+ inflate_radius），按距离机器人排序后保留
   * max_obstacles_ 个。
   */
  void dynamicScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (!enable_dynamic_obstacles_) {
      return;
    }

    // 坐标系校验：ObstacleAdapter 将 scan 视为“传感器位于 base 原点”的极坐标，
    // 因此 scan 的 frame 必须与 base_frame 一致（或已对齐），否则障碍位置会偏移
    if (!msg->header.frame_id.empty()) {
      auto strip = [](const std::string& s) {
        return (!s.empty() && s.front() == '/') ? s.substr(1) : s;
      };
      if (strip(msg->header.frame_id) != strip(base_frame_)) {
        ROS_WARN_THROTTLE(10.0,
                          "AL-iLQR: scan frame_id='%s' differs from base_frame='%s'; "
                          "obstacle positions assume the scan origin is at base_frame.",
                          msg->header.frame_id.c_str(), base_frame_.c_str());
      }
    }

    // 获取机器人在 odom 下的位姿
    // TF lookup 是线程安全的，可直接在回调线程中使用
    double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
    bool pose_ok = getRobotPose(robot_x, robot_y, robot_yaw);

    if (!pose_ok) {
      // TF 失败时回退到里程计数据
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_odom_) {
        ROS_WARN_THROTTLE(2.0, "AL-iLQR: dynamic scan received but no odom for transform");
        return;
      }
      robot_x = latest_odom_.pose.pose.position.x;
      robot_y = latest_odom_.pose.pose.position.y;
      robot_yaw = tf2::getYaw(latest_odom_.pose.pose.orientation);
    }

    // 使用 ObstacleAdapter 进行聚类（极坐标→笛卡尔→聚类→外接圆）
    auto obstacles =
        dynamic_obstacle_adapter_.fromLaserScan(*msg, robot_x, robot_y, robot_yaw);

    // 按距离机器人排序，保留最近的 max_obstacles_ 个
    std::sort(obstacles.begin(), obstacles.end(),
              [robot_x, robot_y](const ObstacleAdapter::CircleObstacle& a,
                                 const ObstacleAdapter::CircleObstacle& b) {
                const double da =
                    (a.x - robot_x) * (a.x - robot_x) + (a.y - robot_y) * (a.y - robot_y);
                const double db =
                    (b.x - robot_x) * (b.x - robot_x) + (b.y - robot_y) * (b.y - robot_y);
                return da < db;
              });

    if (static_cast<int>(obstacles.size()) > max_obstacles_) {
      obstacles.resize(static_cast<size_t>(max_obstacles_));
    }

    // 剔除半径非正/非有限的簇：MultiCircleVehicleObstacleConstraint 的
    // 构造函数对 radius <= 0 会抛异常，必须在进入求解器前过滤
    obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(),
                                   [](const ObstacleAdapter::CircleObstacle& o) {
                                     return !std::isfinite(o.x) || !std::isfinite(o.y) ||
                                            !std::isfinite(o.radius) || o.radius <= 1e-3;
                                   }),
                    obstacles.end());

    // 线程安全存储
    size_t obs_count;
    {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      dynamic_obstacles_ = std::move(obstacles);
      has_dynamic_obstacles_ = !dynamic_obstacles_.empty();
      dynamic_obstacles_stamp_ =
          msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
      obs_count = dynamic_obstacles_.size();
    }

    ROS_DEBUG("AL-iLQR: dynamic scan -> %zu dynamic obstacles", obs_count);
  }

  /**
   * @brief 主规划循环。
   */
  void planningLoop(const ros::TimerEvent& /*event*/) {
    // 复制传感器数据，避免长时间持有锁
    nav_msgs::Odometry odom;
    nav_msgs::Path path;
    ros::Time odom_time;
    double min_scan_range = std::numeric_limits<double>::infinity();
    bool has_odom, has_path, has_scan;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom = latest_odom_;
      path = latest_path_;
      odom_time = last_odom_time_;
      min_scan_range = min_scan_range_;
      has_odom = has_odom_;
      has_path = has_path_;
      has_scan = has_scan_;
    }

    // 开启了动态避障却没有激光数据：属于危险配置（无障碍约束盲跑），必须告警
    if (enable_dynamic_obstacles_ && !has_scan) {
      ROS_WARN_THROTTLE(5.0,
                        "AL-iLQR: dynamic obstacle avoidance enabled but no scan received on "
                        "'%s' yet -- planning without obstacle constraints.",
                        dynamic_scan_topic_.c_str());
    }

    if (!has_odom) {
      publishStatus("WAITING_FOR_ODOM");
      publishZeroVelocity();
      return;
    }

    // 里程计新鲜度检查：数据陈旧说明上游 SLAM/驱动掉线，必须停车
    if (odom_timeout_ > 0.0) {
      const double age = (ros::Time::now() - odom_time).toSec();
      if (age > odom_timeout_) {
        ROS_WARN_THROTTLE(1.0, "AL-iLQR: odometry stale (%.2f s > %.2f s), stopping.",
                          age, odom_timeout_);
        publishStatus("ODOM_TIMEOUT");
        publishZeroVelocity();
        warm_start_controls_.clear();
        return;
      }
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

    // 数值安全：状态必须有限，否则直接停车（避免 NaN 进入求解器）
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !std::isfinite(speed)) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: non-finite robot state (x=%.3f y=%.3f yaw=%.3f v=%.3f), "
                             "stopping.", x, y, yaw, speed);
      publishStatus("INVALID_STATE");
      publishZeroVelocity();
      warm_start_controls_.clear();
      return;
    }
    // 速度反馈限幅到动力学可行范围，防止上游速度尖峰使速度约束一开局即违约
    speed = std::max(vehicle_config_.model.min_speed,
                     std::min(vehicle_config_.model.max_speed, speed));

    if (!has_path) {
      publishStatus("WAITING_FOR_PATH");
      publishZeroVelocity();
      return;
    }

    // ---- 坐标系统一：/initial_path 通常发布在 map 系（Odin map_planner），
    // 而求解器/控制在 odom 系工作。此处按最新 TF 将路径变换到 odom 系，
    // 变换失败则停车（用错坐标系的路径会让机器人朝错误方向猛冲）。
    nav_msgs::Path path_odom;
    if (!transformPathToOdom(path, &path_odom)) {
      publishStatus("PATH_TF_FAILED");
      publishZeroVelocity();
      warm_start_controls_.clear();
      return;
    }
    if (path_odom.poses.size() < 2) {
      publishStatus("PATH_TOO_SHORT");
      publishZeroVelocity();
      return;
    }

    // 到达判定：机器人当前位置与全局路径最后一个点（odom 系）的距离 < tolerance
    {
      const auto& goal_pose = path_odom.poses.back().pose;
      const double goal_dist = std::hypot(goal_pose.position.x - x, goal_pose.position.y - y);
      bool heading_ok = true;
      double heading_err = 0.0;
      if (goal_check_heading_) {
        heading_err = normalizeAngle(tf2::getYaw(goal_pose.orientation) - yaw);
        heading_ok = std::abs(heading_err) < goal_heading_tolerance_;
      }
      if (goal_dist < goal_position_tolerance_ && heading_ok) {
        publishZeroVelocity();
        publishStatus("GOAL_REACHED");
        warm_start_controls_.clear();
        if (publish_arrive_) {
          bool need_publish = false;
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!arrive_published_) {
              arrive_published_ = true;  // 同一条路径只发一次，新路径到来后复位
              need_publish = true;
            }
          }
          if (need_publish) {
            std_msgs::Empty arrive_msg;
            arrive_pub_.publish(arrive_msg);
            ROS_INFO("AL-iLQR: goal reached (dist=%.3f m < %.3f m, heading_err=%.3f rad), "
                     "published arrive on %s",
                     goal_dist, goal_position_tolerance_, heading_err, arrive_topic_.c_str());
          }
        }
        return;
      }
    }

    // 提取 guidance points（reached_end 为辅助信号：采样窗口已覆盖路径末端）
    bool reached_end = false;
    auto guidance = path_converter_.extractGuidance(path_odom, x, y, yaw, speed, &reached_end);
    if (reached_end) {
      ROS_DEBUG_THROTTLE(1.0, "AL-iLQR: guidance window reached path end, "
                              "terminal guidance speed set to 0 for natural deceleration");
    }
    if (static_cast<int>(guidance.size()) < horizon_ + 1) {
      // 终端参考点索引为 horizon，guidance 必须至少 horizon+1 个点；
      // 少于 2 个点则无法定义朝向，直接停车
      if (guidance.size() < 2) {
        ROS_WARN_THROTTLE(1.0, "AL-iLQR: extracted guidance too short (%zu), stopping.",
                          guidance.size());
        publishStatus("GUIDANCE_TOO_SHORT");
        publishZeroVelocity();
        return;
      }
      ROS_DEBUG_THROTTLE(1.0, "AL-iLQR: guidance size %zu < horizon+1 (%d), "
                              "terminal reference clamped to last sample.",
                         guidance.size(), horizon_ + 1);
    }

    // 准备动态障碍物约束（来自动态障碍激光话题，默认 /scan）
    std::vector<my_al_ilqr::DynamicObstacle> dynamic_obstacles;
    if (enable_dynamic_obstacles_) {
      std::vector<ObstacleAdapter::CircleObstacle> raw_obstacles;
      bool stale = false;
      {
        // has_dynamic_obstacles_ 与 dynamic_obstacles_ 必须在同一把锁内读取，
        // 否则与 dynamicScanCallback 的写入构成数据竞争
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        if (has_dynamic_obstacles_) {
          // 障碍物时效校验：激光掉线后不能拿陈旧障碍继续约束（会永久堵死）
          const double age = (ros::Time::now() - dynamic_obstacles_stamp_).toSec();
          if (dynamic_obstacles_timeout_ > 0.0 && age > dynamic_obstacles_timeout_) {
            stale = true;
          } else {
            raw_obstacles = dynamic_obstacles_;
          }
        }
      }
      if (stale) {
        ROS_WARN_THROTTLE(2.0, "AL-iLQR: dynamic obstacles stale (> %.2f s), ignored this cycle.",
                          dynamic_obstacles_timeout_);
      }
      dynamic_obstacles.reserve(raw_obstacles.size());
      for (const auto& obs : raw_obstacles) {
        my_al_ilqr::DynamicObstacle dyn;
        dyn.x = obs.x;
        dyn.y = obs.y;
        dyn.radius = obs.radius;
        dynamic_obstacles.push_back(dyn);
      }
      ROS_DEBUG("AL-iLQR: passing %zu dynamic obstacles to solver",
                dynamic_obstacles.size());
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

    // 调用 GuidanceTracker 求解（传入动态障碍物约束）
    auto result = tracker_->Track(initial_state, guidance, initial_controls,
                                  horizon_, dt_, tracker_config_,
                                  dynamic_obstacles, enable_road_boundary_);

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

    // 数值安全：下发前再确认一次控制量有限
    if (!std::isfinite(accel) || !std::isfinite(steering)) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: non-finite control (a=%.3f, s=%.3f), stopping.",
                        accel, steering);
      publishStatus("INVALID_CONTROL");
      publishZeroVelocity();
      warm_start_controls_.clear();
      return;
    }

    // 映射为差速控制：线速度优先取规划轨迹前视第
    // speed_lookahead_steps_ 步的速度（避免静止起步时 "odom速度+a*dt"
    // 落在底盘死区内的自举死锁）；前视步数为 0 或轨迹无效时回退到
    // 增量积分方式。
    DiffDriveCommand cmd;
    const int look = std::min<int>(speed_lookahead_steps_,
                                   static_cast<int>(result.states.size()) - 1);
    if (look >= 1 && result.states[look].size() >= 4 &&
        std::isfinite(result.states[look](3))) {
      cmd = bicycleToDiffFromSpeed(result.states[look](3), steering,
                                   virtual_wheelbase_, max_linear_vel_,
                                   max_angular_vel_);
    } else {
      // 控制周期用实际规划周期 1/rate，而非求解器 dt
      cmd = bicycleToDiff(accel, steering, speed, control_period_, virtual_wheelbase_,
                          max_linear_vel_, max_angular_vel_);
    }

    if (!std::isfinite(cmd.v_linear) || !std::isfinite(cmd.omega)) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: non-finite cmd_vel, stopping.");
      publishStatus("INVALID_CONTROL");
      publishZeroVelocity();
      warm_start_controls_.clear();
      return;
    }

    // 停滞恢复：求解收敛但速度指令近零，且尚未接近路径末端——
    // 典型为“静止=零违约”局部最优（近零 warm start 自锁）。
    // 连续 stall_recovery_cycles_ 个周期后清空 warm start，
    // 下一周期 GuidanceTracker 会回退到 Pure-Pursuit 初值（含前向
    // 加速度），给求解器一个有前进动力的初始轨迹。
    if (stall_cmd_threshold_ > 0.0 && !reached_end &&
        std::abs(cmd.v_linear) < stall_cmd_threshold_ && ref_speed_ > 1e-3) {
      ++stall_counter_;
      if (stall_counter_ >= stall_recovery_cycles_) {
        ROS_WARN_THROTTLE(2.0,
                          "AL-iLQR: near-zero cmd (v=%.3f m/s) for %d cycles while far from "
                          "goal -- clearing warm start to escape standstill local optimum.",
                          cmd.v_linear, stall_counter_);
        warm_start_controls_.clear();
        stall_counter_ = 0;
      }
    } else {
      stall_counter_ = 0;
    }

    // 发布控制指令
    publishCmdVel(cmd.v_linear, cmd.omega);

    // 发布可视化轨迹
    publishOptimizedPath(guidance, result.states);

    // 发布状态（min_r = /scan 最近有效距离，仅用于诊断）
    char status_msg[320];
    std::snprintf(status_msg, sizeof(status_msg),
                  "OK | cvg=%d | iter=(%d/%d) | %.1fms | %s | viol=%.3f | obs=%zu | "
                  "min_r=%.2f | a=%.3f | s=%.3f | v=%.3f | w=%.3f",
                  static_cast<int>(result.converged),
                  result.outer_iterations,
                  result.total_inner_iterations,
                  result.solve_time_ms,
                  derivative_mode_.c_str(),
                  result.max_violation,
                  dynamic_obstacles.size(),
                  std::isfinite(min_scan_range) ? min_scan_range : -1.0,
                  accel,
                  steering,
                  cmd.v_linear,
                  cmd.omega);
    publishStatus(status_msg);

    ROS_DEBUG("AL-iLQR: %s", status_msg);
  }

  /**
   * @brief 将全局路径从其 header.frame_id 变换到 odom 系。
   *
   * Odin map_planner 发布的 /initial_path 位于 map 系（坐标可达几十米），
   * 而本节点的状态/控制/障碍物均在 odom 系。不做变换直接使用会使
   * 跟踪目标与当前位姿相差一个 map->odom 偏置，表现为不收敛/猛冲。
   *
   * @return true 变换成功（或本来就在 odom 系）
   */
  bool transformPathToOdom(const nav_msgs::Path& in, nav_msgs::Path* out) {
    if (out == nullptr) {
      return false;
    }

    const std::string& src_frame = in.header.frame_id;

    // frame_id 为空：无法判断坐标系，假定已在 odom 系并高频告警
    if (src_frame.empty()) {
      ROS_WARN_THROTTLE(5.0, "AL-iLQR: path has empty frame_id, assuming it is already in '%s'.",
                        odom_frame_.c_str());
      *out = in;
      out->header.frame_id = odom_frame_;
      return true;
    }

    // 兼容带/不带前导 "/" 的 frame 命名
    auto strip = [](const std::string& s) {
      return (!s.empty() && s.front() == '/') ? s.substr(1) : s;
    };
    if (strip(src_frame) == strip(odom_frame_)) {
      *out = in;
      return true;
    }

    geometry_msgs::TransformStamped tf_src_to_odom;
    try {
      // 使用 Time(0)（最新可用变换）：map->odom 由 SLAM 低频发布，
      // 按 path.header.stamp 查询往往外推失败
      tf_src_to_odom = tf_buffer_.lookupTransform(odom_frame_, src_frame, ros::Time(0),
                                                  ros::Duration(0.05));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "AL-iLQR: path TF '%s' -> '%s' failed: %s",
                        src_frame.c_str(), odom_frame_.c_str(), ex.what());
      return false;
    }

    out->header = in.header;
    out->header.frame_id = odom_frame_;
    out->poses.clear();
    out->poses.reserve(in.poses.size());
    for (const auto& ps : in.poses) {
      geometry_msgs::PoseStamped transformed;
      tf2::doTransform(ps.pose, transformed.pose, tf_src_to_odom);
      transformed.header = out->header;
      if (!std::isfinite(transformed.pose.position.x) ||
          !std::isfinite(transformed.pose.position.y)) {
        continue;
      }
      out->poses.push_back(transformed);
    }

    ROS_INFO_ONCE("AL-iLQR: transforming global path from '%s' to '%s' via TF.",
                  src_frame.c_str(), odom_frame_.c_str());
    return true;
  }

  /**
   * @brief 将角度归一到 [-pi, pi]。
   */
  static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
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
  ros::Subscriber dynamic_scan_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher optimized_path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher arrive_pub_;
  ros::Timer planning_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 传感器数据缓存（均受 state_mutex_ 保护）
  std::mutex state_mutex_;
  nav_msgs::Odometry latest_odom_;
  nav_msgs::Path latest_path_;
  ros::Time last_odom_time_;
  // /scan 最近有效距离（仅用于状态诊断；不缓存整帧 scan，避免
  // 规划循环每周期拷贝上千个 float 与重复聚类）
  double min_scan_range_ = std::numeric_limits<double>::infinity();
  bool has_odom_ = false;
  bool has_path_ = false;
  bool has_scan_ = false;

  // 动态障碍物数据缓存（线程安全）
  std::mutex obstacle_mutex_;
  std::vector<ObstacleAdapter::CircleObstacle> dynamic_obstacles_;
  ros::Time dynamic_obstacles_stamp_;
  bool has_dynamic_obstacles_ = false;

  // 参数
  std::string odom_topic_;
  std::string path_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string base_frame_;
  std::string odom_frame_;
  double planning_rate_ = 10.0;
  double control_period_ = 0.1;   // 实际控制周期 = 1/planning_rate
  double odom_timeout_ = 0.5;    // 里程计数据最大允许年龄 s（<=0 禁用）
  int guidance_lookahead_ = 20;
  std::string derivative_mode_ = "analytical";  // 仅用于日志/状态展示

  // 动态障碍物参数
  bool enable_dynamic_obstacles_ = false;
  std::string dynamic_scan_topic_ = "/scan";
  double obstacle_inflate_radius_ = 0.3;
  int max_obstacles_ = 10;
  double cluster_tolerance_ = 0.3;
  double obstacle_max_range_ = 5.0;
  double min_obstacle_radius_ = 0.05;
  double max_obstacle_radius_ = 2.0;
  double max_cluster_radius_ = 0.4;  // 单簇最大外接圆半径（拆分阈值），<=0 禁用
  double dynamic_obstacles_timeout_ = 1.0;  // 障碍物数据最大年龄 s（<=0 禁用）

  // 车辆与求解器
  my_al_ilqr::VehicleBicycleConfig vehicle_config_;
  my_al_ilqr::GuidanceTrackerConfig tracker_config_;
  std::unique_ptr<my_al_ilqr::VehicleCircleApproximation> vehicle_geometry_;
  std::unique_ptr<my_al_ilqr::GuidanceTracker> tracker_;

  // 适配器
  PathConverter path_converter_;
  ObstacleAdapter dynamic_obstacle_adapter_;

  // 求解参数
  int horizon_ = 20;
  double dt_ = 0.1;
  double ref_speed_ = 1.0;
  double road_lower_bound_ = -5.0;
  double road_upper_bound_ = 5.0;
  // 道路边界约束开关（默认禁用：绝对坐标系 y 向走廊，非真车道线）
  bool enable_road_boundary_ = false;

  // 控制映射
  double virtual_wheelbase_ = 0.4;
  double max_linear_vel_ = 1.0;
  double max_angular_vel_ = 1.5;
  // 速度指令前视步数：cmd_vel 线速度取规划轨迹 states[k](3)，
  // 0 = 退化回增量积分（odom速度 + a*dt）
  int speed_lookahead_steps_ = 3;

  // 停滞恢复（仅规划定时器线程访问，无需加锁）
  double stall_cmd_threshold_ = 0.05;  // 速度指令绝对值低于此视为停滞，<=0 禁用
  int stall_recovery_cycles_ = 3;      // 连续停滞周期数阈值
  int stall_counter_ = 0;

  // 到达判定
  double goal_position_tolerance_ = 0.3;
  double goal_heading_tolerance_ = 0.2;
  bool goal_check_heading_ = false;
  bool publish_arrive_ = true;
  std::string arrive_topic_ = "/neupan/arrive";
  bool arrive_published_ = false;  // 防重复发布（受 state_mutex_ 保护）；新路径到来后复位

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
