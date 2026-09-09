#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <atomic>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "al/al_ilqr_solver.hpp"
#include "autodrive/convex_polygon.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/guidance_tracker.hpp"
#include "autodrive/pure_pursuit_tracker.hpp"
#include "autodrive/lse_polygon_obstacle_constraint.hpp"
#include "autodrive/multi_circle_vehicle_obstacle_constraint.hpp"
#include "autodrive/polygon_collision_constraint.hpp"
#include "autodrive/sl_dp_planner.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"
#include "visualization/trajectory_writer.hpp"

namespace {

using my_al_ilqr::ALILQRSolver;
using my_al_ilqr::ALILQROuterIterationLog;
using my_al_ilqr::InitControlsMode;
using my_al_ilqr::ObstacleType;
using my_al_ilqr::StaticObstacleScenarioConfig;
using my_al_ilqr::Trajectory;
using my_al_ilqr::VehicleBicycleConfig;
using my_al_ilqr::VehicleCircleApproximation;
using my_al_ilqr::Vector;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kTrajectoryStepDt = 0.1;
constexpr double kTrajectoryStepHz = 1.0 / kTrajectoryStepDt;
constexpr double kAppliedControlPlotWindowSec = 12.0;
constexpr int kAppliedControlHistoryMaxSamples = 300;

struct PlanningOutput {
  bool success = false;
  bool warm_start_reusable = false;
  bool guidance_used = false;
  bool deadline_missed = false;
  bool stale_on_arrival = false;
  int simulation_step_at_request = -1;
  int request_serial = -1;
  std::string status;
  std::string terminal_state;
  bool converged = false;
  double best_violation = 0.0;
  double initial_violation = 0.0;
  double final_violation = 0.0;
  double initial_cost = 0.0;
  double final_cost = 0.0;
  double planning_time_ms = 0.0;
  double guidance_time_ms = 0.0;
  double tracking_time_ms = 0.0;
  double solver_time_ms = 0.0;
  double sim_time_at_request = 0.0;
  double deadline_ms = 0.0;
  int horizon = 0;
  int outer_iterations = 0;
  int total_inner_iterations = 0;
  std::filesystem::path trajectory_csv_path;
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::vector<ALILQROuterIterationLog> outer_logs;
  std::vector<double> initial_x;
  std::vector<double> initial_y;
  std::vector<double> pp_seed_x;
  std::vector<double> pp_seed_y;
  std::vector<double> guidance_x;
  std::vector<double> guidance_y;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
  std::vector<double> accel;
  std::vector<double> kappa;
  std::vector<double> base_cost_history;
  std::vector<double> violation_history;
  std::vector<double> penalty_history;
  my_al_ilqr::GlobalGuidanceMemory guidance_memory;
  int selected_guidance_topology_id = -1;
  std::vector<my_al_ilqr::GuidanceCandidate> guidance_candidates;
  std::vector<my_al_ilqr::LatticeCandidate> lattice_candidates;
  std::vector<my_al_ilqr::DPLatticeNode> dp_lattice_nodes;
};

struct ScenePreview {
  bool valid = false;
  std::string status;
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
};

enum class FrontendMode { kSinglePlan = 0, kSimulation = 1 };

struct SimulationState {
  bool running = false;
  bool step_pending = false;
  bool use_warm_start = false;
  bool warm_start_valid = false;
  int replan_steps = 5;
  int step_count = 0;
  int steps_since_replan = 0;
  int manual_steps_remaining = 0;
  int stale_plan_tolerance_steps = 2;
  int next_request_serial = 0;
  int last_requested_serial = -1;
  int last_applied_serial = -1;
  int accepted_plan_count = 0;
  int dropped_stale_plan_count = 0;
  int dropped_deadline_plan_count = 0;
  double sim_time = 0.0;
  double control_phase_time = 0.0;
  double physics_tick_dt = 0.02;
  double last_planning_time_ms = 0.0;
  double last_guidance_time_ms = 0.0;
  double last_tracking_time_ms = 0.0;
  double last_solver_time_ms = 0.0;
  double last_planning_deadline_ms = 0.0;
  Vector ego_state;
  Vector applied_control;
  bool applied_control_valid = false;
  std::vector<Vector> executing_controls;
  std::vector<Vector> previous_controls;
  my_al_ilqr::GlobalGuidanceMemory guidance_memory;
  std::vector<double> history_x;
  std::vector<double> history_y;
  std::vector<double> applied_control_time_history;
  std::vector<double> applied_accel_history;
  std::vector<double> applied_steer_history;

  std::vector<double> anim_x;
  std::vector<double> anim_y;
  std::vector<double> anim_yaw;
  std::vector<double> anim_speed;
  std::vector<double> anim_accel;
  std::vector<double> anim_steer;
  int anim_idx = 0;
  int anim_total = 0;
  bool animating = false;
  double anim_last_time = 0.0;
};

struct ReferenceTrackingSummary {
  double terminal_abs_lateral = 0.0;
  double terminal_abs_heading = 0.0;
  double tail_mean_abs_lateral = 0.0;
  double tail_mean_abs_heading = 0.0;
  double base_cost = 0.0;
};

struct InitialGuessDecision {
  bool use_warm_start = false;
  std::string reason;
};

struct FrontendObstacle {
  bool enabled = true;
  double x = 0.0;
  double y = 0.0;
  double speed_x = 0.0;
  double speed_y = 0.0;
  double yaw = 0.0;
  int recycle_generation = 0;
};

struct ObstacleSnapshot {
  int index = -1;
  double x = 0.0;
  double y = 0.0;
  double speed_x = 0.0;
  double speed_y = 0.0;
  double yaw = 0.0;
  bool primary = false;
};

struct SimulationPlanningRequest {
  bool valid = false;
  VehicleBicycleConfig vehicle_config;
  StaticObstacleScenarioConfig scenario_config;
  std::vector<FrontendObstacle> obstacles;
  Vector ego_state;
  std::vector<Vector> warm_start_controls;
  bool use_warm_start = false;
  my_al_ilqr::GlobalGuidanceMemory guidance_memory;
  int request_serial = -1;
  int step_count = 0;
  double sim_time = 0.0;
  double deadline_ms = 0.0;
};

struct FrontendState {
  std::array<char, 512> config_path{};
  VehicleBicycleConfig vehicle_config;
  VehicleBicycleConfig vehicle_config_runtime;
  StaticObstacleScenarioConfig scenario_config;
  StaticObstacleScenarioConfig scenario_config_runtime;
  std::vector<FrontendObstacle> obstacles;
  SimulationState sim_runtime;
  std::vector<FrontendObstacle> obstacles_runtime;
  SimulationPlanningRequest planning_request;
  bool planning_request_ready = false;
  bool planning_request_in_flight = false;
  std::mutex simulation_mutex;
  std::atomic<bool> physics_shutdown{false};
  std::thread physics_thread;
  my_al_ilqr::ScenarioPreset active_preset = my_al_ilqr::ScenarioPreset::kStraightRoad;
  FrontendMode mode = FrontendMode::kSimulation;
  std::string status = "Load a config and press Plan.";
  std::string terminal_state;
  std::filesystem::path last_csv_path;
  double last_best_violation = 0.0;
  bool planning = false;
  bool has_result = false;
  bool preview_dirty = true;
  double scene_zoom = 1.0;
  double scene_pan_x = 0.0;
  double scene_pan_y = 0.0;
  bool show_reference_line = true;
  bool show_obstacle = true;
  bool show_initial_guess = true;
  bool show_pp_seed = true;
  bool show_selected_guidance = true;
  bool show_optimized_trajectory = true;
  bool show_vehicle_outline = true;
  bool show_collision_circles = true;
  bool show_guidance_candidates = false;
  bool show_dp_lattice_nodes = false;
  ScenePreview preview;
  PlanningOutput result;
  SimulationState sim;
  bool show_coming_soon_popup = false;
  GLuint tex_steering_wheel = 0;
  GLuint tex_car_top = 0;
};

struct PreparedPlanningScenario {
  my_al_ilqr::AutodriveDemoScenario scenario;
  my_al_ilqr::GlobalGuidanceResult guidance;
  double guidance_time_ms = 0.0;
  double tracking_time_ms = 0.0;
  std::vector<double> pp_seed_x;
  std::vector<double> pp_seed_y;
  std::string selected_init_controls_label = "scenario-default";
};

struct ScenarioSolveResult {
  std::optional<my_al_ilqr::ALILQRResult> result;
};

struct Bounds2D {
  double min_x = 0.0;
  double max_x = 1.0;
  double min_y = 0.0;
  double max_y = 1.0;
};

struct SimulationDisplayState {
  bool valid = false;
  double sim_time = 0.0;
  Vector ego_state;
};

struct DisplayedTrajectoryState {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;
};

struct SignalPlotOptions {
  const std::vector<double>* x_values = nullptr;
  const char* x_label = "step";
  bool scroll_latest = false;
  double scroll_window = 0.0;
  double fixed_min = std::numeric_limits<double>::quiet_NaN();
  double fixed_max = std::numeric_limits<double>::quiet_NaN();
};

bool SliderInputDouble(const char* label,
                       double* value,
                       double min_value,
                       double max_value,
                       const char* format = "%.3f") {
  bool changed = false;
  ImGui::PushID(label);
  const float full_width = ImGui::GetContentRegionAvail().x;
  const float label_width = 120.0f;
  const float input_width = 88.0f;
  const float spacing = ImGui::GetStyle().ItemSpacing.x;
  const float slider_width =
      std::max(80.0f, full_width - label_width - input_width - 2.0f * spacing);
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  ImGui::SameLine(label_width);
  ImGui::SetNextItemWidth(slider_width);
  changed |= ImGui::SliderScalar("##slider", ImGuiDataType_Double, value, &min_value, &max_value, format);
  ImGui::SameLine(0.0f, spacing);
  ImGui::SetNextItemWidth(input_width);
  changed |= ImGui::InputScalar("##input", ImGuiDataType_Double, value, nullptr, nullptr, format);
  ImGui::PopID();
  return changed;
}

double NormalizeAngleUi(double angle) {
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double Clamp01(double value) {
  return std::clamp(value, 0.0, 1.0);
}

const char* InitControlsModeLabel(InitControlsMode mode) {
  switch (mode) {
    case InitControlsMode::kPlanner:
      return "planner-controls";
    case InitControlsMode::kPP:
      return "pp";
    case InitControlsMode::kPPILQR:
      return "pp+ilqr";
  }
  return "scenario-default";
}

std::string InitControlsFallbackLabel(InitControlsMode mode, const char* reason) {
  std::ostringstream ss;
  ss << InitControlsModeLabel(mode)
     << " [fallback: scenario-default, " << reason << "]";
  return ss.str();
}

std::vector<my_al_ilqr::GuidancePoint> BuildTrackingGuidance(
    const std::vector<my_al_ilqr::GuidancePoint>& guidance,
    const VehicleBicycleConfig& vehicle_config) {
  if (guidance.size() < 3) {
    return guidance;
  }

  std::vector<my_al_ilqr::GuidancePoint> shaped = guidance;
  const size_t n = shaped.size();

  std::vector<double> seg_len(n > 0 ? n - 1 : 0, 0.0);
  for (size_t i = 1; i < n; ++i) {
    const double dx = shaped[i].x - shaped[i - 1].x;
    const double dy = shaped[i].y - shaped[i - 1].y;
    seg_len[i - 1] = std::sqrt(dx * dx + dy * dy);
  }
  shaped[0].s = 0.0;
  for (size_t i = 1; i < n; ++i) {
    shaped[i].s = shaped[i - 1].s + seg_len[i - 1];
  }

  std::vector<double> curvature(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double ds = 0.5 * (seg_len[i - 1] + seg_len[i]);
    if (ds <= 1e-6) {
      continue;
    }
    const double dyaw =
        NormalizeAngleUi(shaped[i + 1].yaw - shaped[i - 1].yaw);
    curvature[i] = std::abs(dyaw) / ds;
  }

  const double wheelbase = std::max(vehicle_config.model.wheelbase, 1e-3);
  const double steer_limit = std::max(
      std::abs(vehicle_config.model.min_steering),
      std::abs(vehicle_config.model.max_steering));
  const double vehicle_kappa_limit = std::tan(steer_limit) / wheelbase;

  double nominal_speed = 0.0;
  for (const auto& pt : shaped) {
    nominal_speed = std::max(nominal_speed, pt.speed);
  }
  nominal_speed = std::max(nominal_speed, 0.5);

  const double lateral_accel_limit =
      std::max(0.5, nominal_speed * nominal_speed * vehicle_kappa_limit * 0.6);
  const double longitudinal_accel_limit = std::max(
      0.5, std::max(vehicle_config.model.max_acceleration,
                    -vehicle_config.model.min_acceleration));

  std::vector<double> speed_limit(n, nominal_speed);
  for (size_t i = 0; i < n; ++i) {
    if (curvature[i] > 1e-4) {
      speed_limit[i] = std::min(
          shaped[i].speed,
          std::sqrt(lateral_accel_limit / curvature[i]));
    } else {
      speed_limit[i] = shaped[i].speed;
    }
    speed_limit[i] = std::clamp(
        speed_limit[i],
        std::max(0.3, vehicle_config.model.min_speed),
        std::max(vehicle_config.model.max_speed, 0.3));
  }

  for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
    const double ds = seg_len[static_cast<size_t>(i)];
    const double v_next = speed_limit[static_cast<size_t>(i + 1)];
    const double v_cap =
        std::sqrt(v_next * v_next + 2.0 * longitudinal_accel_limit * ds);
    speed_limit[static_cast<size_t>(i)] =
        std::min(speed_limit[static_cast<size_t>(i)], v_cap);
  }
  for (size_t i = 1; i < n; ++i) {
    const double ds = seg_len[i - 1];
    const double v_prev = speed_limit[i - 1];
    const double v_cap =
        std::sqrt(v_prev * v_prev + 2.0 * longitudinal_accel_limit * ds);
    speed_limit[i] = std::min(speed_limit[i], v_cap);
  }

  for (size_t i = 0; i < n; ++i) {
    shaped[i].speed = speed_limit[i];
  }
  shaped[0].t = 0.0;
  for (size_t i = 1; i < n; ++i) {
    const double avg_v = 0.5 * (speed_limit[i - 1] + speed_limit[i]);
    shaped[i].t = shaped[i - 1].t + seg_len[i - 1] / std::max(avg_v, 0.3);
  }
  return shaped;
}

double GuidanceTerminalLongitudinalPosition(
    const std::shared_ptr<my_al_ilqr::StraightReferenceLine>& reference_line,
    const std::vector<my_al_ilqr::GuidancePoint>& guidance,
    double fallback) {
  if (!reference_line || guidance.empty()) {
    return fallback;
  }
  Vector point(2);
  point << guidance.back().x, guidance.back().y;
  return reference_line->LongitudinalPosition(point);
}

SimulationDisplayState EvaluateSimulationDisplayState(const FrontendState& state,
                                                      double /*now*/) {
  SimulationDisplayState display;
  if (state.sim.ego_state.size() < 4) {
    return display;
  }

  display.valid = true;
  display.sim_time = state.sim.sim_time;
  display.ego_state = state.sim.ego_state;
  return display;
}

DisplayedTrajectoryState BuildDisplayedTrajectoryState(
    const PlanningOutput* output,
    const SimulationState& sim,
    const SimulationDisplayState& sim_display,
    bool sim_mode) {
  DisplayedTrajectoryState displayed;
  if (output == nullptr || output->x.empty()) {
    return displayed;
  }

  if (!sim_mode || !sim_display.valid || output->x.size() < 2) {
    displayed.x = output->x;
    displayed.y = output->y;
    displayed.yaw = output->yaw;
    displayed.speed = output->speed;
    return displayed;
  }

  if (sim.anim_x.size() >= 2 &&
      sim.anim_y.size() == sim.anim_x.size() &&
      sim.anim_yaw.size() == sim.anim_x.size() &&
      sim.anim_speed.size() == sim.anim_x.size()) {
    const int start_idx = std::clamp(sim.anim_idx + 1, 1,
                                     static_cast<int>(sim.anim_x.size()) - 1);
    const size_t remaining = sim.anim_x.size() - static_cast<size_t>(start_idx) + 1;
    displayed.x.reserve(remaining);
    displayed.y.reserve(remaining);
    displayed.yaw.reserve(remaining);
    displayed.speed.reserve(remaining);

    displayed.x.push_back(sim_display.ego_state(0));
    displayed.y.push_back(sim_display.ego_state(1));
    displayed.yaw.push_back(sim_display.ego_state(2));
    displayed.speed.push_back(sim_display.ego_state(3));
    for (int k = start_idx; k < static_cast<int>(sim.anim_x.size()); ++k) {
      displayed.x.push_back(sim.anim_x[k]);
      displayed.y.push_back(sim.anim_y[k]);
      displayed.yaw.push_back(sim.anim_yaw[k]);
      displayed.speed.push_back(sim.anim_speed[k]);
    }
    return displayed;
  }

  const int start_idx = std::clamp(sim.anim_idx + 1, 1,
                                   static_cast<int>(output->x.size()) - 1);
  const size_t remaining = output->x.size() - static_cast<size_t>(start_idx) + 1;
  displayed.x.reserve(remaining);
  displayed.y.reserve(remaining);
  displayed.yaw.reserve(remaining);
  displayed.speed.reserve(remaining);

  displayed.x.push_back(sim_display.ego_state(0));
  displayed.y.push_back(sim_display.ego_state(1));
  displayed.yaw.push_back(sim_display.ego_state(2));
  displayed.speed.push_back(sim_display.ego_state(3));
  for (int k = start_idx; k < static_cast<int>(output->x.size()); ++k) {
    displayed.x.push_back(output->x[k]);
    displayed.y.push_back(output->y[k]);
    displayed.yaw.push_back(output->yaw[k]);
    displayed.speed.push_back(output->speed[k]);
  }
  return displayed;
}

GLuint LoadTextureFromFile(const char* filename) {
  int w, h, channels;
  unsigned char* data = stbi_load(filename, &w, &h, &channels, 4);
  if (!data) return 0;
  GLuint tex;
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
  stbi_image_free(data);
  return tex;
}

void AddImageRotated(ImDrawList* draw_list, ImTextureID tex,
                     ImVec2 center, float half_w, float half_h, float angle,
                     ImU32 tint = IM_COL32(255,255,255,255)) {
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  auto rot = [&](float lx, float ly) -> ImVec2 {
    return ImVec2(center.x + lx * c - ly * s,
                  center.y + lx * s + ly * c);
  };
  const ImVec2 p0 = rot(-half_w, -half_h);
  const ImVec2 p1 = rot( half_w, -half_h);
  const ImVec2 p2 = rot( half_w,  half_h);
  const ImVec2 p3 = rot(-half_w,  half_h);
  draw_list->AddImageQuad(tex, p0, p1, p2, p3,
                          ImVec2(0,0), ImVec2(1,0), ImVec2(1,1), ImVec2(0,1), tint);
}

ImVec2 ToCanvas(const ImVec2& origin, const ImVec2& size, const Bounds2D& bounds, double x, double y) {
  const double span_x = std::max(1e-6, bounds.max_x - bounds.min_x);
  const double span_y = std::max(1e-6, bounds.max_y - bounds.min_y);
  const float px = origin.x + static_cast<float>((x - bounds.min_x) / span_x) * size.x;
  const float py = origin.y + size.y - static_cast<float>((y - bounds.min_y) / span_y) * size.y;
  return ImVec2(px, py);
}

double ObstacleHalfSpanX(const StaticObstacleScenarioConfig& config) {
  if (config.obstacle_type == ObstacleType::kCircle) {
    return config.obstacle_radius;
  }
  const double c = std::cos(config.obstacle_yaw);
  const double s = std::sin(config.obstacle_yaw);
  return std::abs(config.obstacle_half_length * c) +
         std::abs(config.obstacle_half_width * s);
}

double ObstacleHalfSpanY(const StaticObstacleScenarioConfig& config) {
  if (config.obstacle_type == ObstacleType::kCircle) {
    return config.obstacle_radius;
  }
  const double c = std::cos(config.obstacle_yaw);
  const double s = std::sin(config.obstacle_yaw);
  return std::abs(config.obstacle_half_length * s) +
         std::abs(config.obstacle_half_width * c);
}

double EffectiveObstacleRadius(const StaticObstacleScenarioConfig& config) {
  if (config.obstacle_type == ObstacleType::kCircle) {
    return config.obstacle_radius;
  }
  return std::sqrt(ObstacleHalfSpanX(config) * ObstacleHalfSpanX(config) +
                   ObstacleHalfSpanY(config) * ObstacleHalfSpanY(config));
}

double ClampObstacleLateral(const StaticObstacleScenarioConfig& config,
                            double y) {
  const double limit =
      std::max(0.4, config.road_half_width - ObstacleHalfSpanY(config) - 0.45);
  return std::clamp(y, -limit, limit);
}

double Hash01(int a, int b) {
  const double value = std::sin(
      static_cast<double>(a) * 12.9898 +
      static_cast<double>(b) * 78.233) * 43758.5453;
  return value - std::floor(value);
}

double DefaultObstacleGap(const StaticObstacleScenarioConfig& config,
                          int obstacle_index,
                          int generation) {
  const double min_gap =
      std::max(11.0, 5.0 * ObstacleHalfSpanX(config) + 5.0);
  const double max_gap = min_gap + 10.0;
  return min_gap + (max_gap - min_gap) *
      Hash01(101 + obstacle_index * 17, 211 + generation * 29);
}

double DefaultObstacleLateral(const StaticObstacleScenarioConfig& config,
                              int obstacle_index,
                              int generation) {
  const double limit =
      std::max(0.4, config.road_half_width - ObstacleHalfSpanY(config) - 0.45);
  const double selector =
      Hash01(307 + obstacle_index * 13, 401 + generation * 23);
  double y = 0.0;
  if (selector < 0.3) {
    y = (Hash01(509 + obstacle_index * 19, 601 + generation * 31) - 0.5) *
        0.7 * limit;
  } else {
    const double side = selector < 0.65 ? -1.0 : 1.0;
    const double amp = 0.45 + 0.45 *
        Hash01(701 + obstacle_index * 11, 809 + generation * 37);
    y = side * amp * limit;
  }
  return ClampObstacleLateral(config, y);
}

double DefaultObstacleSpeed(const StaticObstacleScenarioConfig& config,
                            int obstacle_index) {
  const double min_speed = -1.5;
  const double max_speed = std::max(1.0, 0.7 * config.target_speed);
  return min_speed + (max_speed - min_speed) *
      Hash01(907 + obstacle_index * 7, 1009 + obstacle_index * 11);
}

int FindPrimaryObstacleIndex(const std::vector<FrontendObstacle>& obstacles,
                             double reference_x) {
  int best_ahead_index = -1;
  double best_ahead_distance = std::numeric_limits<double>::infinity();
  int best_any_index = -1;
  double best_any_distance = std::numeric_limits<double>::infinity();

  for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
    if (!obstacles[i].enabled) {
      continue;
    }
    const double obstacle_x = obstacles[i].x;
    const double dx = obstacle_x - reference_x;
    const double abs_dx = std::abs(dx);
    if (abs_dx < best_any_distance) {
      best_any_distance = abs_dx;
      best_any_index = i;
    }
    if (dx >= -1.0 && dx < best_ahead_distance) {
      best_ahead_distance = dx;
      best_ahead_index = i;
    }
  }

  return best_ahead_index >= 0 ? best_ahead_index : best_any_index;
}

std::vector<ObstacleSnapshot> BuildObstacleSnapshots(
    const std::vector<FrontendObstacle>& obstacles,
    double reference_x) {
  std::vector<ObstacleSnapshot> snapshots;
  const int primary_index = FindPrimaryObstacleIndex(obstacles, reference_x);
  snapshots.reserve(obstacles.size());
  for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
    if (!obstacles[i].enabled) {
      continue;
    }
    snapshots.push_back(ObstacleSnapshot{
        i,
        obstacles[i].x,
        obstacles[i].y,
        obstacles[i].speed_x,
        obstacles[i].speed_y,
        obstacles[i].yaw,
        i == primary_index,
    });
  }
  return snapshots;
}

std::vector<my_al_ilqr::PredictedObstacle> BuildObstaclePredictions(
    const StaticObstacleScenarioConfig& config,
    const std::vector<FrontendObstacle>& obstacles,
    int horizon,
    double dt) {
  std::vector<my_al_ilqr::PredictedObstacle> predictions;
  predictions.reserve(obstacles.size());
  for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
    if (!obstacles[i].enabled) {
      continue;
    }
    if (config.obstacle_type == ObstacleType::kCircle) {
      predictions.push_back(my_al_ilqr::MakeConstantVelocityPrediction(
          i, config.obstacle_radius, obstacles[i].x,
          obstacles[i].y, obstacles[i].speed_x, obstacles[i].speed_y,
          horizon, dt));
    } else {
      predictions.push_back(my_al_ilqr::MakeBoxConstantVelocityPrediction(
          i, config.obstacle_half_length, config.obstacle_half_width,
          obstacles[i].yaw, obstacles[i].x,
          obstacles[i].y, obstacles[i].speed_x, obstacles[i].speed_y,
          horizon, dt));
    }
  }
  return predictions;
}

my_al_ilqr::GlobalGuidanceConfig BuildGuidanceConfig(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& scenario_config,
    const std::vector<FrontendObstacle>& obstacles) {
  my_al_ilqr::GlobalGuidanceConfig config;
  const double max_lateral_span =
      std::max(0.5, scenario_config.road_half_width - ObstacleHalfSpanY(scenario_config) - 0.2);
  const double lateral_span =
      std::clamp(scenario_config.guidance_search_lateral_span, 0.4, max_lateral_span);

  auto add_lateral_offset = [&](std::vector<double>* offsets, double value) {
    if (offsets == nullptr) {
      return;
    }
    offsets->push_back(std::clamp(value, -lateral_span, lateral_span));
  };

  std::vector<double> lateral_offsets;
  int sample_count = std::max(5, scenario_config.guidance_search_lateral_samples);
  if ((sample_count % 2) == 0) {
    sample_count += 1;
  }
  lateral_offsets.reserve(static_cast<size_t>(sample_count) + obstacles.size() * 6 + 8);
  if (sample_count == 1) {
    lateral_offsets.push_back(0.0);
  } else {
    for (int i = 0; i < sample_count; ++i) {
      const double ratio =
          static_cast<double>(i) / static_cast<double>(sample_count - 1);
      add_lateral_offset(&lateral_offsets, -lateral_span + 2.0 * lateral_span * ratio);
    }
  }

  const double obstacle_clearance =
      ObstacleHalfSpanY(scenario_config) + 0.5 * vehicle_config.body.width + 0.35;
  std::vector<std::pair<double, double>> blocked_intervals;
  blocked_intervals.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    if (!obstacle.enabled) {
      continue;
    }
    add_lateral_offset(&lateral_offsets, obstacle.y);
    add_lateral_offset(&lateral_offsets, obstacle.y - 0.5 * obstacle_clearance);
    add_lateral_offset(&lateral_offsets, obstacle.y + 0.5 * obstacle_clearance);
    add_lateral_offset(&lateral_offsets, obstacle.y - obstacle_clearance);
    add_lateral_offset(&lateral_offsets, obstacle.y + obstacle_clearance);

    blocked_intervals.emplace_back(
        std::max(-lateral_span, obstacle.y - obstacle_clearance),
        std::min(lateral_span, obstacle.y + obstacle_clearance));
  }

  std::sort(blocked_intervals.begin(), blocked_intervals.end());
  std::vector<std::pair<double, double>> merged_intervals;
  for (const auto& interval : blocked_intervals) {
    if (merged_intervals.empty() || interval.first > merged_intervals.back().second) {
      merged_intervals.push_back(interval);
    } else {
      merged_intervals.back().second =
          std::max(merged_intervals.back().second, interval.second);
    }
  }

  auto add_gap_samples = [&](double start, double end) {
    const double width = end - start;
    if (width < 0.2) {
      return;
    }
    add_lateral_offset(&lateral_offsets, 0.5 * (start + end));
    if (width > 0.9) {
      add_lateral_offset(&lateral_offsets, start + 0.25 * width);
      add_lateral_offset(&lateral_offsets, start + 0.75 * width);
    }
  };

  double free_start = -lateral_span;
  for (const auto& interval : merged_intervals) {
    add_gap_samples(free_start, interval.first);
    free_start = std::max(free_start, interval.second);
  }
  add_gap_samples(free_start, lateral_span);

  lateral_offsets.push_back(0.0);
  std::sort(lateral_offsets.begin(), lateral_offsets.end());
  lateral_offsets.erase(
      std::unique(lateral_offsets.begin(), lateral_offsets.end(),
                  [](double lhs, double rhs) { return std::abs(lhs - rhs) < 0.08; }),
      lateral_offsets.end());

  const int max_offsets = std::max(sample_count, 9);
  if (static_cast<int>(lateral_offsets.size()) > max_offsets) {
    std::vector<double> compacted;
    compacted.reserve(max_offsets);
    compacted.push_back(lateral_offsets.front());
    if (max_offsets > 1) {
      for (int i = 1; i < max_offsets - 1; ++i) {
        const double ratio =
            static_cast<double>(i) / static_cast<double>(max_offsets - 1);
        const size_t idx = static_cast<size_t>(std::llround(
            ratio * static_cast<double>(lateral_offsets.size() - 1)));
        compacted.push_back(lateral_offsets[std::min(idx, lateral_offsets.size() - 1)]);
      }
      compacted.push_back(lateral_offsets.back());
    }
    compacted.push_back(0.0);
    std::sort(compacted.begin(), compacted.end());
    compacted.erase(
        std::unique(compacted.begin(), compacted.end(),
                    [](double lhs, double rhs) { return std::abs(lhs - rhs) < 0.08; }),
        compacted.end());
    lateral_offsets = std::move(compacted);
  }
  config.lateral_offsets = std::move(lateral_offsets);

  config.speed_ratios = {0.0, 0.5, 1.0, std::max(1.0, scenario_config.guidance_search_max_speed_ratio)};
  config.num_layers = scenario_config.guidance_search_layers;
  config.lateral_weight = scenario_config.guidance_search_lateral_weight;
  config.speed_weight = scenario_config.guidance_search_speed_weight;
  config.accel_weight = scenario_config.guidance_search_accel_weight;
  config.steering_weight = scenario_config.guidance_search_steering_weight;
  config.path_length_weight = scenario_config.guidance_search_path_length_weight;
  config.collision_penalty = scenario_config.guidance_search_collision_penalty;
  config.proximity_buffer = scenario_config.guidance_search_proximity_buffer;
  config.proximity_weight = scenario_config.guidance_search_proximity_weight;
  config.follow_lateral_band = scenario_config.guidance_search_follow_lateral_band;
  config.follow_base_gap = scenario_config.guidance_search_follow_base_gap;
  config.follow_headway = scenario_config.guidance_search_follow_headway;
  config.follow_penalty = scenario_config.guidance_search_follow_penalty;
  config.bypass_lateral_clearance =
      scenario_config.guidance_search_bypass_lateral_clearance;
  config.bypass_anticipation_distance =
      scenario_config.guidance_search_bypass_anticipation_distance;
  config.bypass_anticipation_weight =
      scenario_config.guidance_search_bypass_anticipation_weight;
  config.post_bypass_hold_distance =
      scenario_config.guidance_search_post_bypass_hold_distance;
  config.post_bypass_hold_weight =
      scenario_config.guidance_search_post_bypass_hold_weight;
  config.consistency_penalty = scenario_config.guidance_search_consistency_penalty;
  config.consistency_geometry_weight =
      scenario_config.guidance_search_consistency_geometry_weight;
  config.min_acceleration = vehicle_config.model.min_acceleration;
  config.max_acceleration = vehicle_config.model.max_acceleration;
  config.min_steering = vehicle_config.model.min_steering;
  config.max_steering = vehicle_config.model.max_steering;
  return config;
}

my_al_ilqr::SLDPConfig BuildSLDPConfig(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& scenario_config) {
  my_al_ilqr::SLDPConfig config;
  config.num_layers = scenario_config.dp_layers;
  config.lateral_samples = scenario_config.dp_lateral_samples;
  config.lateral_span = scenario_config.dp_lateral_span;
  config.speed_samples = scenario_config.dp_speed_samples;
  config.max_speed_ratio = scenario_config.dp_max_speed_ratio;
  config.max_lateral_index_jump = scenario_config.dp_max_lateral_jump;
  config.lateral_weight = scenario_config.dp_lateral_weight;
  config.lateral_change_weight = scenario_config.dp_lateral_change_weight;
  config.heading_weight = scenario_config.dp_heading_weight;
  config.speed_weight = scenario_config.dp_speed_weight;
  config.accel_weight = scenario_config.dp_accel_weight;
  config.steering_weight = scenario_config.dp_steering_weight;
  config.path_length_weight = scenario_config.dp_path_length_weight;
  config.collision_penalty = scenario_config.dp_collision_penalty;
  config.proximity_buffer = scenario_config.dp_proximity_buffer;
  config.proximity_weight = scenario_config.dp_proximity_weight;
  config.anticipation_distance = scenario_config.dp_anticipation_distance;
  config.anticipation_lateral_clearance = scenario_config.dp_anticipation_lateral_clearance;
  config.anticipation_weight = scenario_config.dp_anticipation_weight;
  config.min_acceleration = vehicle_config.model.min_acceleration;
  config.max_acceleration = vehicle_config.model.max_acceleration;
  config.min_steering = vehicle_config.model.min_steering;
  config.max_steering = vehicle_config.model.max_steering;
  return config;
}

Vector BuildInitialStateVector(const StaticObstacleScenarioConfig& config) {
  Vector initial_state(4);
  initial_state << config.initial_x, config.initial_y, config.initial_yaw, config.initial_speed;
  return initial_state;
}

std::vector<my_al_ilqr::LatticeCandidate> ConvertGuidanceCandidatesForView(
    const std::vector<my_al_ilqr::GuidanceCandidate>& candidates) {
  std::vector<my_al_ilqr::LatticeCandidate> converted;
  converted.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    my_al_ilqr::LatticeCandidate view;
    view.x.reserve(candidate.points.size());
    view.y.reserve(candidate.points.size());
    for (const auto& point : candidate.points) {
      view.x.push_back(point.x);
      view.y.push_back(point.y);
    }
    view.cost = candidate.cost;
    view.collision_free = candidate.collision_free;
    view.is_best = candidate.selected;
    converted.push_back(std::move(view));
  }
  return converted;
}

const my_al_ilqr::GuidanceCandidate* SelectedGuidanceCandidate(const PlanningOutput* output) {
  if (output == nullptr) {
    return nullptr;
  }
  for (const auto& candidate : output->guidance_candidates) {
    if (candidate.selected) {
      return &candidate;
    }
  }
  return output->guidance_candidates.empty() ? nullptr : &output->guidance_candidates.front();
}

const my_al_ilqr::TopologyInteraction* FindTopologyInteraction(
    const my_al_ilqr::TopologySignature& signature,
    int obstacle_id) {
  for (const auto& interaction : signature.interactions) {
    if (interaction.obstacle_id == obstacle_id) {
      return &interaction;
    }
  }
  return nullptr;
}

const char* LateralRelationLabel(int lateral_relation) {
  if (lateral_relation > 0) {
    return "R";
  }
  if (lateral_relation < 0) {
    return "L";
  }
  return "?";
}

const char* TemporalRelationLabel(int temporal_relation) {
  if (temporal_relation > 0) {
    return "B";
  }
  if (temporal_relation < 0) {
    return "A";
  }
  return "?";
}

std::string FormatTopologyInteractionShort(
    const my_al_ilqr::TopologyInteraction& interaction) {
  std::ostringstream oss;
  oss << LateralRelationLabel(interaction.lateral_relation) << "/"
      << TemporalRelationLabel(interaction.temporal_relation);
  return oss.str();
}

std::string FormatTopologySignatureReadable(
    const my_al_ilqr::TopologySignature& signature) {
  if (signature.interactions.empty()) {
    return "<empty>";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < signature.interactions.size(); ++i) {
    if (i > 0) {
      oss << " | ";
    }
    oss << "Obs" << signature.interactions[i].obstacle_id << ": "
        << FormatTopologyInteractionShort(signature.interactions[i]);
  }
  return oss.str();
}

std::string FormatSignatureKeyReadable(const std::string& key) {
  if (key.empty()) {
    return "<none>";
  }

  std::ostringstream oss;
  size_t token_start = 0;
  bool first = true;
  while (token_start < key.size()) {
    const size_t token_end = key.find('|', token_start);
    const std::string token = key.substr(
        token_start,
        token_end == std::string::npos ? std::string::npos : token_end - token_start);
    const size_t colon = token.find(':');
    if (colon != std::string::npos && colon + 2 < token.size()) {
      if (!first) {
        oss << " | ";
      }
      oss << "Obs" << token.substr(0, colon) << ": " << token[colon + 1] << "/"
          << token[colon + 2];
      first = false;
    }
    if (token_end == std::string::npos) {
      break;
    }
    token_start = token_end + 1;
  }

  const std::string readable = oss.str();
  return readable.empty() ? key : readable;
}

void DrawRelationLegendTable() {
  ImGui::SeparatorText("Relation Legend");
  if (ImGui::BeginTable("relation_legend", 2,
                        ImGuiTableFlags_BordersInnerV |
                            ImGuiTableFlags_RowBg |
                            ImGuiTableFlags_SizingStretchSame)) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted("R / L");
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted("Lateral side in Frenet sign convention: R=+d, L=-d");

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted("A / B");
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted("Temporal relation: A=After, B=Before");

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted("Example");
    ImGui::TableSetColumnIndex(1);
    ImGui::TextUnformatted("R/A = positive-lateral side and passes after that obstacle");
    ImGui::EndTable();
  }
}

void ClampObstacleFieldToRoad(const StaticObstacleScenarioConfig& config,
                              std::vector<FrontendObstacle>* obstacles) {
  if (!obstacles || !config.enable_obstacle_recycling) {
    return;
  }
  for (auto& obstacle : *obstacles) {
    obstacle.y = ClampObstacleLateral(config, obstacle.y);
  }
}

void InitializeObstacleField(std::vector<FrontendObstacle>* obstacles,
                             const StaticObstacleScenarioConfig& config,
                             int count = 3) {
  if (!obstacles) {
    return;
  }
  obstacles->clear();
  obstacles->reserve(count);
  double next_x = std::max(6.0, config.obstacle_center_x) + 2.0;
  for (int i = 0; i < count; ++i) {
    FrontendObstacle obstacle;
    obstacle.enabled = true;
    obstacle.recycle_generation = 0;
    if (i == 0) {
      obstacle.x = next_x;
      obstacle.y = ClampObstacleLateral(config, config.obstacle_center_y);
      obstacle.speed_x = config.obstacle_speed_x;
    } else {
      next_x += DefaultObstacleGap(config, i, 0);
      obstacle.x = next_x;
      obstacle.y = DefaultObstacleLateral(config, i, 0);
      obstacle.speed_x = DefaultObstacleSpeed(config, i);
    }
    obstacles->push_back(obstacle);
  }
}

FrontendObstacle MakeObstacle(double x, double y, double speed_x = 0.0,
                              double speed_y = 0.0, double yaw = 0.0) {
  FrontendObstacle obs;
  obs.enabled = true;
  obs.x = x;
  obs.y = y;
  obs.speed_x = speed_x;
  obs.speed_y = speed_y;
  obs.yaw = yaw;
  obs.recycle_generation = 0;
  return obs;
}

void ApplyScenarioPreset(my_al_ilqr::ScenarioPreset preset,
                         StaticObstacleScenarioConfig* config,
                         std::vector<FrontendObstacle>* obstacles) {
  if (!config || !obstacles) return;

  *config = StaticObstacleScenarioConfig{};
  obstacles->clear();

  using P = my_al_ilqr::ScenarioPreset;
  switch (preset) {
    case P::kStraightRoad: {
      InitializeObstacleField(obstacles, *config);
      break;
    }

    case P::kStaggeredObstacles: {
      config->road_half_width = 4.5;
      config->target_speed = 4.0;
      config->initial_speed = 2.5;
      config->enable_obstacle_recycling = false;
      const double lat_limit = config->road_half_width - 1.2;
      const int count = 4;
      double next_x = 12.0;
      for (int i = 0; i < count; ++i) {
        next_x += 8.0 + 14.0 * Hash01(131 + i * 17, 257);
        const double r = Hash01(311 + i * 13, 419);
        double y;
        if (r < 0.25) {
          y = (Hash01(503 + i * 19, 613) - 0.5) * 1.2;
        } else {
          y = (Hash01(701 + i * 11, 811) * 2.0 - 1.0) * lat_limit;
        }
        obstacles->push_back(MakeObstacle(next_x, y));
      }
      config->obstacle_center_x = obstacles->front().x;
      config->obstacle_center_y = obstacles->front().y;
      break;
    }

    case P::kOvertaking: {
      const double lane_w = 3.5;
      config->road_half_width = lane_w;
      config->ref_origin_y = -lane_w / 2.0;
      config->num_lanes = 2;
      config->lane_width = lane_w;
      config->target_speed = 5.0;
      config->initial_x = 0.0;
      config->initial_y = -lane_w / 2.0;
      config->initial_yaw = 0.0;
      config->initial_speed = 3.0;
      config->enable_obstacle_recycling = false;
      obstacles->push_back(MakeObstacle(20.0, -lane_w / 2.0, 1.5));
      obstacles->push_back(MakeObstacle(55.0, lane_w / 2.0, -4.0, 0.0, M_PI));
      config->obstacle_center_x = obstacles->front().x;
      config->obstacle_center_y = obstacles->front().y;
      config->obstacle_speed_x = obstacles->front().speed_x;
      break;
    }

    case P::kIntersection: {
      config->road_half_width = 4.5;
      config->target_speed = 4.0;
      config->initial_speed = 3.0;
      config->enable_obstacle_recycling = false;
      config->cross_road = {true, 30.0, 4.0};
      obstacles->push_back(MakeObstacle(30.0, 25.0, 0.0, -3.0, M_PI / 2.0));
      config->obstacle_center_x = obstacles->front().x;
      config->obstacle_center_y = obstacles->front().y;
      break;
    }

    case P::kCarFollowing: {
      config->road_half_width = 2.0;
      config->target_speed = 4.0;
      config->initial_speed = 2.5;
      config->enable_obstacle_recycling = false;
      obstacles->push_back(MakeObstacle(12.0, 0.0, 1.5));
      config->obstacle_center_x = obstacles->front().x;
      config->obstacle_center_y = obstacles->front().y;
      config->obstacle_speed_x = obstacles->front().speed_x;
      break;
    }
  }
}

StaticObstacleScenarioConfig ScenarioConfigForObstacleField(
    const StaticObstacleScenarioConfig& base_config,
    const std::vector<FrontendObstacle>& obstacles,
    double reference_x) {
  StaticObstacleScenarioConfig config = base_config;
  const int primary_index = FindPrimaryObstacleIndex(obstacles, reference_x);
  if (primary_index >= 0) {
    const auto& obstacle = obstacles[primary_index];
    config.obstacle_center_x = obstacle.x;
    config.obstacle_center_y = obstacle.y;
    config.obstacle_speed_x = obstacle.speed_x;
  }
  return config;
}

template <class ExpandFn>
void ExpandObstacleBoundsAt(const StaticObstacleScenarioConfig& config,
                            double x,
                            double y,
                            ExpandFn&& expand) {
  expand(x - ObstacleHalfSpanX(config), y - ObstacleHalfSpanY(config));
  expand(x + ObstacleHalfSpanX(config), y + ObstacleHalfSpanY(config));
}

void AdvanceObstacleWorld(const StaticObstacleScenarioConfig& config,
                          double ego_x,
                          double dt,
                          std::vector<FrontendObstacle>* obstacles) {
  if (!obstacles) {
    return;
  }
  const double recycle_threshold = ego_x - 16.0;
  double farthest_x = ego_x + 10.0;
  for (auto& obstacle : *obstacles) {
    if (!obstacle.enabled) {
      continue;
    }
    obstacle.x += obstacle.speed_x * dt;
    obstacle.y += obstacle.speed_y * dt;
    farthest_x = std::max(farthest_x, obstacle.x);
  }

  if (!config.enable_obstacle_recycling) {
    return;
  }

  for (int i = 0; i < static_cast<int>(obstacles->size()); ++i) {
    auto& obstacle = (*obstacles)[i];
    if (!obstacle.enabled) {
      continue;
    }
    if (obstacle.x >= recycle_threshold) {
      continue;
    }
    obstacle.recycle_generation += 1;
    farthest_x += DefaultObstacleGap(config, i, obstacle.recycle_generation);
    obstacle.x = farthest_x;
    obstacle.y = DefaultObstacleLateral(config, i, obstacle.recycle_generation);
    obstacle.speed_y = 0.0;
    obstacle.yaw = 0.0;
  }
}

Bounds2D ComputeSceneBounds(const FrontendState& state, const PlanningOutput* output) {
  const ScenePreview& preview = state.preview;
  Bounds2D bounds;
  bounds.min_x = std::numeric_limits<double>::infinity();
  bounds.max_x = -std::numeric_limits<double>::infinity();
  bounds.min_y = std::numeric_limits<double>::infinity();
  bounds.max_y = -std::numeric_limits<double>::infinity();

  auto expand = [&](double x, double y) {
    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.max_y = std::max(bounds.max_y, y);
  };

  for (size_t i = 0; i < preview.x.size(); ++i) {
    expand(preview.x[i], preview.y[i]);
  }
  if (output) {
    for (size_t i = 0; i < output->x.size(); ++i) {
      expand(output->x[i], output->y[i]);
    }
    for (size_t i = 0; i < output->initial_x.size(); ++i) {
      expand(output->initial_x[i], output->initial_y[i]);
    }
  }

  const double reference_x =
      state.mode == FrontendMode::kSimulation && state.sim.ego_state.size() >= 1
          ? state.sim.ego_state(0)
          : state.scenario_config.initial_x;
  for (const auto& obstacle : BuildObstacleSnapshots(state.obstacles, reference_x)) {
    ExpandObstacleBoundsAt(preview.scenario_config, obstacle.x, obstacle.y, expand);
  }
  expand(0.0, -preview.scenario_config.road_half_width);
  expand(20.0, preview.scenario_config.road_half_width);
  if (preview.scenario_config.cross_road.enabled) {
    const double cx = preview.scenario_config.cross_road.center_x;
    const double chw = preview.scenario_config.cross_road.half_width;
    expand(cx - chw, 0.0);
    expand(cx + chw, 0.0);
  }

  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.max_x)) {
    bounds = Bounds2D{};
  }

  const double margin_x = std::max(1.0, 0.08 * (bounds.max_x - bounds.min_x));
  const double margin_y = std::max(0.8, 0.15 * (bounds.max_y - bounds.min_y));
  bounds.min_x -= margin_x;
  bounds.max_x += margin_x;
  bounds.min_y -= margin_y;
  bounds.max_y += margin_y;
  return bounds;
}

Bounds2D ApplySceneView(const Bounds2D& input_bounds, double zoom, double pan_x, double pan_y) {
  Bounds2D bounds = input_bounds;
  zoom = std::clamp(zoom, 0.25, 20.0);
  const double cx = 0.5 * (bounds.min_x + bounds.max_x) + pan_x;
  const double cy = 0.5 * (bounds.min_y + bounds.max_y) + pan_y;
  const double span_x = std::max(1e-6, (bounds.max_x - bounds.min_x) / zoom);
  const double span_y = std::max(1e-6, (bounds.max_y - bounds.min_y) / zoom);
  bounds.min_x = cx - 0.5 * span_x;
  bounds.max_x = cx + 0.5 * span_x;
  bounds.min_y = cy - 0.5 * span_y;
  bounds.max_y = cy + 0.5 * span_y;
  return bounds;
}

std::vector<ImVec2> MakeObstaclePolyline(const ImVec2& origin,
                                         const ImVec2& size,
                                         const Bounds2D& bounds,
                                         double cx,
                                         double cy,
                                         double radius,
                                         int segments = 48) {
  std::vector<ImVec2> pts;
  pts.reserve(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    const double theta = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
    pts.push_back(ToCanvas(origin, size, bounds,
                           cx + radius * std::cos(theta),
                           cy + radius * std::sin(theta)));
  }
  return pts;
}

std::vector<ImVec2> MakeRectangleObstaclePolyline(const ImVec2& origin,
                                                  const ImVec2& size,
                                                  const Bounds2D& bounds,
                                                  double cx,
                                                  double cy,
                                                  double half_length,
                                                  double half_width,
                                                  double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double corners[][2] = {
      {-half_length, -half_width},
      { half_length, -half_width},
      { half_length,  half_width},
      {-half_length,  half_width},
      {-half_length, -half_width},
  };
  std::vector<ImVec2> pts;
  pts.reserve(5);
  for (const auto& corner : corners) {
    double wx = cx + c * corner[0] - s * corner[1];
    double wy = cy + s * corner[0] + c * corner[1];
    pts.push_back(ToCanvas(origin, size, bounds, wx, wy));
  }
  return pts;
}

std::vector<ImVec2> MakeVehicleOutline(const VehicleBicycleConfig& config,
                                       const ImVec2& origin,
                                       const ImVec2& size,
                                       const Bounds2D& bounds,
                                       double x,
                                       double y,
                                       double yaw) {
  const double rear = config.body.rear_axle_to_rear;
  const double front = config.body.length - config.body.rear_axle_to_rear;
  const double half_width = 0.5 * config.body.width;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  const std::vector<std::pair<double, double>> body_corners = {
      {front, half_width},
      {front, -half_width},
      {-rear, -half_width},
      {-rear, half_width},
      {front, half_width},
  };

  std::vector<ImVec2> polyline;
  polyline.reserve(body_corners.size());
  for (const auto& corner : body_corners) {
    const double wx = x + c * corner.first - s * corner.second;
    const double wy = y + s * corner.first + c * corner.second;
    polyline.push_back(ToCanvas(origin, size, bounds, wx, wy));
  }
  return polyline;
}

void DrawPolyline(ImDrawList* draw_list,
                  const std::vector<ImVec2>& polyline,
                  ImU32 color,
                  float thickness);
void FillClosedPolygon(ImDrawList* draw_list,
                       const std::vector<ImVec2>& polygon,
                       ImU32 fill_color);

void DrawObstacleAt(ImDrawList* draw_list,
                    const StaticObstacleScenarioConfig& config,
                    const ImVec2& origin,
                    const ImVec2& size,
                    const Bounds2D& bounds,
                    double x,
                    double y,
                    ImU32 fill_color,
                    ImU32 line_color,
                    double obs_yaw_override = std::numeric_limits<double>::quiet_NaN()) {
  if (config.obstacle_type == ObstacleType::kCircle) {
    const auto obstacle = MakeObstaclePolyline(
        origin, size, bounds, x, y, config.obstacle_radius);
    FillClosedPolygon(draw_list, obstacle, fill_color);
    DrawPolyline(draw_list, obstacle, line_color, 2.5f);
    return;
  }

  const double yaw = std::isnan(obs_yaw_override) ? config.obstacle_yaw : obs_yaw_override;
  const auto obstacle = MakeRectangleObstaclePolyline(
      origin, size, bounds,
      x, y,
      config.obstacle_half_length,
      config.obstacle_half_width,
      yaw);
  FillClosedPolygon(draw_list, obstacle, fill_color);
  DrawPolyline(draw_list, obstacle, line_color, 2.5f);
}

void DrawOverlayLabel(ImDrawList* draw_list,
                      const ImVec2& anchor,
                      const std::string& text,
                      ImU32 text_color,
                      ImU32 background_color) {
  if (draw_list == nullptr || text.empty()) {
    return;
  }
  const ImVec2 text_size = ImGui::CalcTextSize(text.c_str());
  const ImVec2 pad(5.0f, 3.0f);
  draw_list->AddRectFilled(
      ImVec2(anchor.x - pad.x, anchor.y - pad.y),
      ImVec2(anchor.x + text_size.x + pad.x, anchor.y + text_size.y + pad.y),
      background_color, 4.0f);
  draw_list->AddText(anchor, text_color, text.c_str());
}

void AddObstacleConstraintAtStage(
    my_al_ilqr::AutodriveDemoScenario* scenario,
    const StaticObstacleScenarioConfig& config,
    int stage,
    double x,
    double y,
    double obs_yaw = 0.0) {
  if (!scenario) {
    return;
  }

  switch (config.obstacle_type) {
    case ObstacleType::kCircle:
      scenario->constrained_problem->AddStageConstraint(
          stage, std::make_shared<my_al_ilqr::MultiCircleVehicleObstacleConstraint>(
                     scenario->vehicle_geometry, x, y, config.obstacle_radius));
      break;

    case ObstacleType::kPolygonLSE: {
      auto polygon = my_al_ilqr::MakeRectangle(
          x, y, config.obstacle_half_length, config.obstacle_half_width,
          obs_yaw);
      scenario->constrained_problem->AddStageConstraint(
          stage, std::make_shared<my_al_ilqr::LSEPolygonObstacleConstraint>(
                     scenario->vehicle_geometry, std::move(polygon),
                     config.lse_alpha));
      break;
    }

    case ObstacleType::kPolygonExact: {
      auto polygon = my_al_ilqr::MakeRectangle(
          x, y, config.obstacle_half_length, config.obstacle_half_width,
          obs_yaw);
      scenario->constrained_problem->AddStageConstraint(
          stage, std::make_shared<my_al_ilqr::PolygonCollisionConstraint>(
                     scenario->vehicle_config.body.length,
                     scenario->vehicle_config.body.width,
                     scenario->vehicle_config.body.rear_axle_to_rear,
                     std::move(polygon), 0.1));
      break;
    }
  }
}

void AddObstacleFieldConstraints(
    my_al_ilqr::AutodriveDemoScenario* scenario,
    const StaticObstacleScenarioConfig& config,
    const std::vector<FrontendObstacle>& obstacles,
    double reference_x) {
  if (!scenario || !scenario->base_problem || !scenario->constrained_problem) {
    return;
  }

  const int horizon = scenario->base_problem->Horizon();
  const double dt = scenario->base_problem->TimeStep();
  const int primary_index = FindPrimaryObstacleIndex(obstacles, reference_x);
  for (int k = 0; k < horizon; ++k) {
    for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
      if (!obstacles[i].enabled || i == primary_index) {
        continue;
      }
      const double t = dt * static_cast<double>(k);
      AddObstacleConstraintAtStage(
          scenario, config, k,
          obstacles[i].x + obstacles[i].speed_x * t,
          obstacles[i].y + obstacles[i].speed_y * t,
          obstacles[i].yaw);
    }
  }
}

ScenarioSolveResult SolvePreparedScenarioRobustly(
    const PreparedPlanningScenario& prepared,
    const Vector& initial_state,
    const std::vector<Vector>& initial_controls) {
  if (!prepared.scenario.control_rate_costs.empty()) {
    const int n = static_cast<int>(prepared.scenario.control_rate_costs.size());
    const Vector zero_control = Vector::Zero(2);
    for (int k = 0; k < n; ++k) {
      const Vector& prev = (k == 0) ? zero_control : initial_controls[k - 1];
      prepared.scenario.control_rate_costs[k]->SetPreviousControl(prev);
    }
  }
  ScenarioSolveResult solve_result;
  ALILQRSolver solver(*prepared.scenario.constrained_problem,
                      prepared.scenario.solver_options);
  solve_result.result = solver.Solve(initial_state, initial_controls);
  return solve_result;
}

PreparedPlanningScenario PreparePlanningScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& config,
    const std::vector<FrontendObstacle>& obstacles,
    double reference_x,
    const my_al_ilqr::GlobalGuidanceMemory* previous_memory = nullptr,
    const Vector* initial_state_override = nullptr) {
  StaticObstacleScenarioConfig cfg = config;

  PreparedPlanningScenario prepared;
  prepared.scenario =
      my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, cfg);
  if (initial_state_override != nullptr && initial_state_override->size() == 4) {
    prepared.scenario.initial_state = *initial_state_override;
  }

  const int horizon = prepared.scenario.base_problem->Horizon();
  const double dt = prepared.scenario.base_problem->TimeStep();
  {
    const double ego_x =
        (initial_state_override && initial_state_override->size() == 4)
            ? (*initial_state_override)(0)
            : cfg.initial_x;
    const double ego_v =
        (initial_state_override && initial_state_override->size() == 4)
            ? (*initial_state_override)(3)
            : cfg.initial_speed;
    cfg.target_longitudinal_position = ego_x + ego_v * horizon * dt;
  }
  prepared.scenario =
      my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, cfg);
  if (initial_state_override != nullptr && initial_state_override->size() == 4) {
    prepared.scenario.initial_state = *initial_state_override;
  }

  const bool use_guidance = cfg.init_guess_mode == my_al_ilqr::InitGuessMode::kGlobalGuidance
                         || cfg.init_guess_mode == my_al_ilqr::InitGuessMode::kSLDP;
  if (use_guidance) {
    const Vector planner_initial_state =
        initial_state_override != nullptr && initial_state_override->size() == 4
            ? *initial_state_override
            : BuildInitialStateVector(cfg);
    const std::vector<my_al_ilqr::PredictedObstacle> predictions =
        BuildObstaclePredictions(cfg, obstacles, horizon, dt);
    const double planner_half_width = std::max(
        std::abs(prepared.scenario.road_lower_bound),
        std::abs(prepared.scenario.road_upper_bound));

    const auto t_guid_start = std::chrono::steady_clock::now();
    if (cfg.init_guess_mode == my_al_ilqr::InitGuessMode::kGlobalGuidance) {
      my_al_ilqr::GlobalGuidancePlanner planner(
          prepared.scenario.reference_line, vehicle_config.model.wheelbase);
      prepared.guidance = planner.Generate(
          planner_initial_state, cfg.target_speed, planner_half_width,
          prepared.scenario.vehicle_geometry, predictions, horizon, dt,
          BuildGuidanceConfig(vehicle_config, cfg, obstacles),
          previous_memory);
    } else {
      my_al_ilqr::SLDPPlanner planner(
          prepared.scenario.reference_line, vehicle_config.model.wheelbase);
      prepared.guidance = planner.Generate(
          planner_initial_state, cfg.target_speed, planner_half_width,
          prepared.scenario.vehicle_geometry, predictions, horizon, dt,
          BuildSLDPConfig(vehicle_config, cfg));
    }
    prepared.guidance_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_guid_start).count();

    const std::vector<my_al_ilqr::GuidancePoint>* selected_guidance_display =
        prepared.guidance.selected_guidance.empty() ? nullptr : &prepared.guidance.selected_guidance;
    const std::vector<my_al_ilqr::GuidancePoint> tracking_guidance =
        selected_guidance_display != nullptr
            ? BuildTrackingGuidance(*selected_guidance_display, vehicle_config)
            : std::vector<my_al_ilqr::GuidancePoint>{};
    const std::vector<my_al_ilqr::GuidancePoint>* selected_guidance_tracking =
        tracking_guidance.empty() ? nullptr : &tracking_guidance;

    std::vector<my_al_ilqr::Vector> init_controls_result;
    std::vector<my_al_ilqr::Vector> init_states_result;

    if (cfg.init_controls_mode == InitControlsMode::kPlanner) {
      if (!prepared.guidance.controls.empty()) {
        init_controls_result = prepared.guidance.controls;
        prepared.selected_init_controls_label = "planner-controls";
      } else {
        prepared.selected_init_controls_label = InitControlsFallbackLabel(
            cfg.init_controls_mode, "planner returned empty controls");
      }
    } else if (selected_guidance_tracking != nullptr) {
      const int H = prepared.scenario.base_problem->Horizon();
      const double step_dt = prepared.scenario.base_problem->TimeStep();

      auto pp_dynamics = std::make_shared<my_al_ilqr::KinematicBicycleModel>(
          vehicle_config.model.wheelbase);
      const double pp_steer_margin = 1.5;
      const double pp_accel_margin = 2.0;
      my_al_ilqr::Vector pp_lb(2), pp_ub(2);
      pp_lb << vehicle_config.model.min_acceleration * pp_accel_margin,
               vehicle_config.model.min_steering * pp_steer_margin;
      pp_ub << vehicle_config.model.max_acceleration * pp_accel_margin,
               vehicle_config.model.max_steering * pp_steer_margin;

      auto pp_result = my_al_ilqr::PurePursuitTracker::Track(
          tracking_guidance,
          prepared.scenario.initial_state,
          pp_dynamics, pp_lb, pp_ub,
          H, step_dt, vehicle_config.model.wheelbase);

      if (!pp_result.states.empty()) {
        prepared.pp_seed_x.resize(pp_result.states.size());
        prepared.pp_seed_y.resize(pp_result.states.size());
        for (size_t i = 0; i < pp_result.states.size(); ++i) {
          prepared.pp_seed_x[i] = pp_result.states[i](0);
          prepared.pp_seed_y[i] = pp_result.states[i](1);
        }
      }

      if (cfg.init_controls_mode == InitControlsMode::kPP) {
        init_controls_result = std::move(pp_result.controls);
        init_states_result = std::move(pp_result.states);
        char pp_label[64];
        std::snprintf(pp_label, sizeof(pp_label), "pp (err=%.2fm)",
                      pp_result.max_pos_error);
      } else if (cfg.init_controls_mode == InitControlsMode::kPPILQR) {
        my_al_ilqr::GuidanceTracker tracker(
            vehicle_config,
            prepared.scenario.road_lower_bound,
            prepared.scenario.road_upper_bound,
            prepared.scenario.vehicle_geometry);

        my_al_ilqr::GuidanceTrackerConfig track_cfg;
        track_cfg.position_weight = cfg.guidance_position_weight;
        track_cfg.heading_weight = cfg.guidance_heading_weight;
        track_cfg.speed_weight = cfg.guidance_speed_weight;
        track_cfg.terminal_position_weight = cfg.guidance_terminal_position_weight;
        track_cfg.terminal_heading_weight = cfg.guidance_terminal_heading_weight;
        track_cfg.terminal_speed_weight = cfg.guidance_terminal_speed_weight;
        track_cfg.accel_reg_weight = cfg.stage_accel_weight;
        track_cfg.steering_reg_weight = cfg.stage_steering_weight;

        auto track_result = tracker.Track(
            prepared.scenario.initial_state,
            tracking_guidance,
            pp_result.controls,
            H, step_dt, track_cfg);

        prepared.tracking_time_ms = track_result.solve_time_ms;
        init_controls_result = std::move(track_result.controls);
        init_states_result = std::move(track_result.states);
        prepared.selected_init_controls_label = "pp+ilqr";
      }
    } else {
      prepared.selected_init_controls_label = InitControlsFallbackLabel(
          cfg.init_controls_mode, "no selected guidance");
    }

    std::vector<my_al_ilqr::GuidancePoint> init_traj_guidance;
    const std::vector<my_al_ilqr::GuidancePoint>* longitudinal_target_guidance =
        selected_guidance_tracking;

    if (!init_states_result.empty()) {
      init_traj_guidance.reserve(init_states_result.size());
      double cumulative_s = 0.0;
      for (size_t i = 0; i < init_states_result.size(); ++i) {
        const auto& st = init_states_result[i];
        my_al_ilqr::GuidancePoint gp;
        gp.x = st(0);
        gp.y = st(1);
        gp.yaw = st(2);
        gp.speed = st(3);
        gp.t = static_cast<double>(i) * dt;
        if (i > 0) {
          const auto& prev = init_states_result[i - 1];
          double dx = st(0) - prev(0);
          double dy = st(1) - prev(1);
          cumulative_s += std::sqrt(dx * dx + dy * dy);
        }
        gp.s = cumulative_s;
        init_traj_guidance.push_back(gp);
      }
      longitudinal_target_guidance = &init_traj_guidance;
    }

    if (longitudinal_target_guidance != nullptr) {
      cfg.target_longitudinal_position = GuidanceTerminalLongitudinalPosition(
          prepared.scenario.reference_line, *longitudinal_target_guidance,
          cfg.target_longitudinal_position);
    }

    if (cfg.use_guidance_tracking && selected_guidance_tracking != nullptr) {
      cfg.target_longitudinal_position = GuidanceTerminalLongitudinalPosition(
          prepared.scenario.reference_line, tracking_guidance,
          cfg.target_longitudinal_position);
      prepared.scenario =
          my_al_ilqr::CreateSingleStaticObstacleTestScenario(
              vehicle_config, cfg, selected_guidance_tracking);
      if (initial_state_override != nullptr && initial_state_override->size() == 4) {
        prepared.scenario.initial_state = *initial_state_override;
      }
    } else if (longitudinal_target_guidance != nullptr) {
      prepared.scenario =
          my_al_ilqr::CreateSingleStaticObstacleTestScenario(vehicle_config, cfg);
      if (initial_state_override != nullptr && initial_state_override->size() == 4) {
        prepared.scenario.initial_state = *initial_state_override;
      }
    }

    if (!init_controls_result.empty()) {
      prepared.scenario.initial_controls = std::move(init_controls_result);
    }
    prepared.scenario.selected_guidance = prepared.guidance.selected_guidance;
    prepared.scenario.selected_guidance_topology_id = prepared.guidance.selected_topology_id;
    prepared.scenario.guidance_candidates = prepared.guidance.candidate_bundle;
    prepared.scenario.lattice_candidates =
        ConvertGuidanceCandidatesForView(prepared.guidance.candidate_bundle);
    prepared.scenario.dp_lattice_nodes = prepared.guidance.dp_lattice_nodes;
  }

  AddObstacleFieldConstraints(
      &prepared.scenario, cfg, obstacles, reference_x);
  return prepared;
}

void DrawPolyline(ImDrawList* draw_list, const std::vector<ImVec2>& polyline, ImU32 color, float thickness) {
  if (!draw_list || polyline.size() < 2) {
    return;
  }
  draw_list->AddPolyline(polyline.data(), static_cast<int>(polyline.size()), color, 0, thickness);
}

void FillClosedPolygon(ImDrawList* draw_list, const std::vector<ImVec2>& polygon, ImU32 fill_color) {
  if (!draw_list || polygon.size() < 3) {
    return;
  }
  const int count = static_cast<int>(polygon.size()) - 1;
  if (count >= 3) {
    draw_list->AddConvexPolyFilled(polygon.data(), count, fill_color);
  }
}

void DrawDashedPolyline(ImDrawList* draw_list,
                        const std::vector<ImVec2>& polyline,
                        ImU32 color,
                        float thickness,
                        int draw_stride = 2) {
  if (!draw_list || polyline.size() < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    if ((static_cast<int>(i) % draw_stride) != 0) {
      continue;
    }
    draw_list->AddLine(polyline[i], polyline[i + 1], color, thickness);
  }
}

ImU32 BlendSpeedColor(double speed, double min_speed, double max_speed, int alpha = 220) {
  const double normalized =
      Clamp01((speed - min_speed) / std::max(1e-6, max_speed - min_speed));
  const ImVec4 slow(0.10f, 0.78f, 0.96f, 1.0f);
  const ImVec4 cruise(0.18f, 0.86f, 0.44f, 1.0f);
  const ImVec4 fast(1.00f, 0.72f, 0.18f, 1.0f);
  const ImVec4 very_fast(1.00f, 0.37f, 0.24f, 1.0f);

  auto lerp = [](const ImVec4& a, const ImVec4& b, double t) {
    const float tf = static_cast<float>(Clamp01(t));
    return ImVec4(a.x + (b.x - a.x) * tf,
                  a.y + (b.y - a.y) * tf,
                  a.z + (b.z - a.z) * tf,
                  a.w + (b.w - a.w) * tf);
  };

  ImVec4 color;
  if (normalized < 0.4) {
    color = lerp(slow, cruise, normalized / 0.4);
  } else if (normalized < 0.78) {
    color = lerp(cruise, fast, (normalized - 0.4) / 0.38);
  } else {
    color = lerp(fast, very_fast, (normalized - 0.78) / 0.22);
  }
  return IM_COL32(static_cast<int>(255.0f * color.x),
                  static_cast<int>(255.0f * color.y),
                  static_cast<int>(255.0f * color.z),
                  alpha);
}

void DrawTrajectoryRibbon(ImDrawList* draw_list,
                          const VehicleBicycleConfig& vehicle_config,
                          const ImVec2& origin,
                          const ImVec2& size,
                          const Bounds2D& bounds,
                          const std::vector<double>& x,
                          const std::vector<double>& y,
                          const std::vector<double>& speed,
                          double min_speed,
                          double max_speed) {
  if (!draw_list || x.size() < 2 || y.size() != x.size() || speed.size() != x.size()) {
    return;
  }

  const double half_width = std::max(0.28, 0.42 * vehicle_config.body.width);
  std::vector<ImVec2> left;
  std::vector<ImVec2> right;
  left.reserve(x.size());
  right.reserve(x.size());

  for (size_t i = 0; i < x.size(); ++i) {
    double tx = 1.0;
    double ty = 0.0;
    if (i == 0) {
      tx = x[1] - x[0];
      ty = y[1] - y[0];
    } else if (i + 1 == x.size()) {
      tx = x[i] - x[i - 1];
      ty = y[i] - y[i - 1];
    } else {
      tx = x[i + 1] - x[i - 1];
      ty = y[i + 1] - y[i - 1];
    }
    const double norm = std::max(1e-6, std::sqrt(tx * tx + ty * ty));
    tx /= norm;
    ty /= norm;
    const double nx = -ty;
    const double ny = tx;

    left.push_back(ToCanvas(origin, size, bounds,
                            x[i] + half_width * nx,
                            y[i] + half_width * ny));
    right.push_back(ToCanvas(origin, size, bounds,
                             x[i] - half_width * nx,
                             y[i] - half_width * ny));
  }

  for (size_t i = 0; i + 1 < x.size(); ++i) {
    const double segment_speed = 0.5 * (speed[i] + speed[i + 1]);
    const ImU32 fill = BlendSpeedColor(segment_speed, min_speed, max_speed, 170);
    const ImU32 edge = BlendSpeedColor(segment_speed, min_speed, max_speed, 235);
    const ImVec2 quad[4] = {left[i], left[i + 1], right[i + 1], right[i]};
    draw_list->AddConvexPolyFilled(quad, 4, fill);
    draw_list->AddLine(left[i], left[i + 1], edge, 1.1f);
    draw_list->AddLine(right[i], right[i + 1], edge, 1.1f);
  }
}

double ComputeNiceTickStep(double span, int target_intervals = 4) {
  if (!std::isfinite(span) || span <= 0.0) {
    return 1.0;
  }
  const double raw_step = span / static_cast<double>(std::max(1, target_intervals));
  const double exponent = std::floor(std::log10(raw_step));
  const double scale = std::pow(10.0, exponent);
  const double fraction = raw_step / scale;
  double nice_fraction = 1.0;
  if (fraction <= 1.0) {
    nice_fraction = 1.0;
  } else if (fraction <= 2.0) {
    nice_fraction = 2.0;
  } else if (fraction <= 5.0) {
    nice_fraction = 5.0;
  } else {
    nice_fraction = 10.0;
  }
  return nice_fraction * scale;
}

int TickLabelDecimals(double step) {
  if (!std::isfinite(step) || step <= 0.0) {
    return 0;
  }
  int decimals = 0;
  double scaled = std::abs(step);
  while (scaled < 1.0 && decimals < 4) {
    scaled *= 10.0;
    ++decimals;
  }
  return decimals;
}

void DrawSignalPlot(const char* title,
                    const std::vector<double>& values,
                    ImU32 color,
                    const SignalPlotOptions& options = {}) {
  ImGui::BeginChild(title, ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar);
  ImGui::TextUnformatted(title);
  const ImVec2 top_left = ImGui::GetCursorScreenPos();
  const ImVec2 canvas_size = ImVec2(std::max(80.0f, ImGui::GetContentRegionAvail().x),
                                    std::max(100.0f, ImGui::GetContentRegionAvail().y - 8.0f));
  const ImVec2 bottom_right(top_left.x + canvas_size.x, top_left.y + canvas_size.y);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(19, 23, 31, 255), 6.0f);
  draw_list->AddRect(top_left, bottom_right, IM_COL32(70, 78, 96, 255), 6.0f, 0, 1.0f);

  if (!values.empty()) {
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < values.size(); ++i) {
      const double x = options.x_values != nullptr && options.x_values->size() == values.size()
                           ? (*options.x_values)[i]
                           : static_cast<double>(i);
      const double value = values[i];
      if (!std::isfinite(x) || !std::isfinite(value)) {
        continue;
      }
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
    if (std::isfinite(options.fixed_min)) {
      min_value = options.fixed_min;
    }
    if (std::isfinite(options.fixed_max)) {
      max_value = options.fixed_max;
    }
    if (!std::isfinite(min_x) || !std::isfinite(max_x)) {
      min_x = 0.0;
      max_x = 1.0;
    }
    if (!std::isfinite(min_value) || !std::isfinite(max_value)) {
      min_value = -1.0;
      max_value = 1.0;
    }
    if (options.scroll_latest && options.scroll_window > 0.0 && std::isfinite(max_x)) {
      min_x = std::max(0.0, max_x - options.scroll_window);
    }
    if (std::abs(max_x - min_x) < 1e-6) {
      max_x += 1.0;
    }
    if (std::abs(max_value - min_value) < 1e-6) {
      min_value -= 1.0;
      max_value += 1.0;
    }

    const float pad_left = 46.0f;
    const float pad_right = 14.0f;
    const float pad_top = 12.0f;
    const float pad_bottom = 28.0f;
    const ImVec2 plot_min(top_left.x + pad_left, top_left.y + pad_top);
    const ImVec2 plot_max(bottom_right.x - pad_right, bottom_right.y - pad_bottom);
    const float plot_w = std::max(1.0f, plot_max.x - plot_min.x);
    const float plot_h = std::max(1.0f, plot_max.y - plot_min.y);

    const double x_step = ComputeNiceTickStep(max_x - min_x);
    const double y_step = ComputeNiceTickStep(max_value - min_value);
    const int x_decimals = TickLabelDecimals(x_step);
    const int y_decimals = TickLabelDecimals(y_step);

    const double start_x_tick = std::ceil(min_x / x_step) * x_step;
    for (double tick = start_x_tick; tick <= max_x + 0.5 * x_step; tick += x_step) {
      const float t = static_cast<float>((tick - min_x) / (max_x - min_x));
      if (t < -1e-4f || t > 1.0001f) {
        continue;
      }
      const float px = plot_min.x + t * plot_w;
      draw_list->AddLine(ImVec2(px, plot_min.y), ImVec2(px, plot_max.y),
                         IM_COL32(54, 61, 77, 255), 1.0f);
      draw_list->AddLine(ImVec2(px, plot_max.y), ImVec2(px, plot_max.y + 4.0f),
                         IM_COL32(120, 128, 146, 255), 1.0f);
      char label[32];
      const double shown_tick = std::abs(tick) < 1e-9 ? 0.0 : tick;
      std::snprintf(label, sizeof(label), "%.*f", x_decimals, shown_tick);
      const ImVec2 text_size = ImGui::CalcTextSize(label);
      draw_list->AddText(ImVec2(px - 0.5f * text_size.x, plot_max.y + 6.0f),
                         IM_COL32(184, 192, 208, 255), label);
    }

    const double start_y_tick = std::ceil(min_value / y_step) * y_step;
    for (double tick = start_y_tick; tick <= max_value + 0.5 * y_step; tick += y_step) {
      const float t = static_cast<float>((tick - min_value) / (max_value - min_value));
      if (t < -1e-4f || t > 1.0001f) {
        continue;
      }
      const float py = plot_max.y - t * plot_h;
      draw_list->AddLine(ImVec2(plot_min.x, py), ImVec2(plot_max.x, py),
                         IM_COL32(54, 61, 77, 255), 1.0f);
      draw_list->AddLine(ImVec2(plot_min.x - 4.0f, py), ImVec2(plot_min.x, py),
                         IM_COL32(120, 128, 146, 255), 1.0f);
      char label[32];
      const double shown_tick = std::abs(tick) < 1e-9 ? 0.0 : tick;
      std::snprintf(label, sizeof(label), "%.*f", y_decimals, shown_tick);
      const ImVec2 text_size = ImGui::CalcTextSize(label);
      draw_list->AddText(ImVec2(plot_min.x - 8.0f - text_size.x, py - 0.5f * text_size.y),
                         IM_COL32(184, 192, 208, 255), label);
    }

    if (min_value <= 0.0 && max_value >= 0.0) {
      const float zero_t = static_cast<float>((0.0 - min_value) / (max_value - min_value));
      const float zero_y = plot_max.y - zero_t * plot_h;
      draw_list->AddLine(ImVec2(plot_min.x, zero_y), ImVec2(plot_max.x, zero_y),
                         IM_COL32(98, 106, 126, 255), 1.2f);
    }

    draw_list->AddRect(plot_min, plot_max, IM_COL32(98, 106, 126, 255), 0.0f, 0, 1.0f);

    draw_list->PushClipRect(plot_min, plot_max, true);
    std::vector<ImVec2> segment;
    segment.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
      const double x = options.x_values != nullptr && options.x_values->size() == values.size()
                           ? (*options.x_values)[i]
                           : static_cast<double>(i);
      const double y = values[i];
      if (!std::isfinite(x) || !std::isfinite(y) || x < min_x || x > max_x) {
        if (segment.size() >= 2) {
          DrawPolyline(draw_list, segment, color, 2.0f);
        }
        segment.clear();
        continue;
      }
      const float tx = static_cast<float>((x - min_x) / (max_x - min_x));
      const float ty = static_cast<float>((y - min_value) / (max_value - min_value));
      segment.emplace_back(plot_min.x + tx * plot_w,
                           plot_max.y - ty * plot_h);
    }
    if (segment.size() >= 2) {
      DrawPolyline(draw_list, segment, color, 2.0f);
    }
    draw_list->PopClipRect();

    const ImVec2 x_label_size = ImGui::CalcTextSize(options.x_label);
    draw_list->AddText(ImVec2(plot_max.x - x_label_size.x, bottom_right.y - 20.0f),
                       IM_COL32(190, 198, 215, 255), options.x_label);
  } else {
    draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f), IM_COL32(180, 180, 180, 255), "No data");
  }

  ImGui::Dummy(canvas_size);
  ImGui::EndChild();
}

void DrawLegendEntry(ImDrawList* draw_list,
                     const ImVec2& anchor,
                     ImU32 color,
                     const char* label,
                     bool enabled,
                     bool dashed = false) {
  const ImVec2 p0(anchor.x, anchor.y + 6.0f);
  const ImVec2 p1(anchor.x + 18.0f, anchor.y + 6.0f);
  if (enabled) {
    if (dashed) {
      draw_list->AddLine(p0, ImVec2(anchor.x + 8.0f, anchor.y + 6.0f), color, 2.0f);
      draw_list->AddLine(ImVec2(anchor.x + 10.0f, anchor.y + 6.0f), p1, color, 2.0f);
    } else {
      draw_list->AddLine(p0, p1, color, 2.0f);
    }
  } else {
    draw_list->AddLine(p0, p1, IM_COL32(90, 90, 90, 120), 1.0f);
  }
  draw_list->AddText(ImVec2(anchor.x + 24.0f, anchor.y), enabled ? IM_COL32(230, 230, 230, 255)
                                                                 : IM_COL32(120, 120, 120, 180),
                     label);
}

void DrawSceneView(FrontendState* state) {
  if (!state || !state->preview.valid) {
    return;
  }
  const ScenePreview& preview = state->preview;
  const PlanningOutput* output = state->has_result ? &state->result : nullptr;
  const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 canvas_size(std::max(200.0f, avail.x), std::max(220.0f, avail.y));
  const ImVec2 canvas_max(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  draw_list->AddRectFilled(canvas_pos, canvas_max, IM_COL32(15, 18, 24, 255), 8.0f);
  draw_list->AddRect(canvas_pos, canvas_max, IM_COL32(65, 72, 88, 255), 8.0f, 0, 1.0f);

  const float pad = 18.0f;
  const ImVec2 plot_origin(canvas_pos.x + pad, canvas_pos.y + pad);
  const ImVec2 plot_size(canvas_size.x - 2.0f * pad, canvas_size.y - 2.0f * pad);

  const bool sim_mode = state->mode == FrontendMode::kSimulation;
  const SimulationDisplayState sim_display =
      sim_mode ? EvaluateSimulationDisplayState(*state, glfwGetTime())
               : SimulationDisplayState{};
  const DisplayedTrajectoryState displayed_plan =
      BuildDisplayedTrajectoryState(output, state->sim, sim_display, sim_mode);

  const bool hovered = ImGui::IsMouseHoveringRect(canvas_pos, canvas_max, true);
  if (hovered) {
    ImGuiIO& io = ImGui::GetIO();
    if (std::abs(io.MouseWheel) > 1e-6f) {
      const double zoom_factor = io.MouseWheel > 0.0f ? 1.12 : 1.0 / 1.12;
      state->scene_zoom = std::clamp(state->scene_zoom * zoom_factor, 0.25, 20.0);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      const double scale_est = 30.0 / std::max(1.0f, plot_size.x) / state->scene_zoom;
      state->scene_pan_x -= static_cast<double>(ImGui::GetIO().MouseDelta.x) * scale_est;
      state->scene_pan_y += static_cast<double>(ImGui::GetIO().MouseDelta.y) * scale_est;
    }
  }

  const double obstacle_reference_x =
      sim_mode && sim_display.valid
          ? sim_display.ego_state(0)
          : (sim_mode && state->sim.ego_state.size() >= 1
                 ? state->sim.ego_state(0)
                 : state->scenario_config.initial_x);
  const auto obstacles = BuildObstacleSnapshots(
      state->obstacles, obstacle_reference_x);
  const auto* selected_candidate = SelectedGuidanceCandidate(output);

  Bounds2D bounds;
  const double rhw = preview.scenario_config.road_half_width;
  if (sim_mode && sim_display.valid && sim_display.ego_state.size() >= 2) {
    const double view_length = 30.0 / state->scene_zoom;
    const double ego_x = sim_display.ego_state(0) + state->scene_pan_x;
    bounds.min_x = ego_x - view_length * 0.25;
    bounds.max_x = ego_x + view_length * 0.75;
    const double cy = state->scene_pan_y;
    bounds.min_y = cy - (rhw + 2.0) / state->scene_zoom;
    bounds.max_y = cy + (rhw + 2.0) / state->scene_zoom;
  } else {
    bounds = ApplySceneView(ComputeSceneBounds(*state, output),
                            state->scene_zoom, state->scene_pan_x, state->scene_pan_y);
  }

  {
    const double rx = bounds.max_x - bounds.min_x;
    const double ry = bounds.max_y - bounds.min_y;
    if (rx > 1e-9 && ry > 1e-9 && plot_size.x > 1.0f && plot_size.y > 1.0f) {
      const double sx = rx / static_cast<double>(plot_size.x);
      const double sy = ry / static_cast<double>(plot_size.y);
      const double scale = std::max(sx, sy);
      const double new_rx = scale * static_cast<double>(plot_size.x);
      const double new_ry = scale * static_cast<double>(plot_size.y);
      const double cx = (bounds.min_x + bounds.max_x) / 2.0;
      const double cy = (bounds.min_y + bounds.max_y) / 2.0;
      bounds.min_x = cx - new_rx / 2.0;
      bounds.max_x = cx + new_rx / 2.0;
      bounds.min_y = cy - new_ry / 2.0;
      bounds.max_y = cy + new_ry / 2.0;
    }
  }

  const auto lower_a = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, -rhw);
  const auto lower_b = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, -rhw);
  const auto upper_a = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, rhw);
  const auto upper_b = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, rhw);

  {
    const auto road_tl = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, rhw);
    const auto road_br = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, -rhw);
    draw_list->AddRectFilled(
        ImVec2(std::min(road_tl.x, road_br.x), std::min(road_tl.y, road_br.y)),
        ImVec2(std::max(road_tl.x, road_br.x), std::max(road_tl.y, road_br.y)),
        IM_COL32(40, 42, 48, 255));
  }

  const auto& cross_road = preview.scenario_config.cross_road;
  if (cross_road.enabled) {
    const double cx = cross_road.center_x;
    const double chw = cross_road.half_width;
    const auto cr_tl = ToCanvas(plot_origin, plot_size, bounds, cx - chw, bounds.max_y);
    const auto cr_br = ToCanvas(plot_origin, plot_size, bounds, cx + chw, bounds.min_y);
    draw_list->AddRectFilled(
        ImVec2(std::min(cr_tl.x, cr_br.x), std::min(cr_tl.y, cr_br.y)),
        ImVec2(std::max(cr_tl.x, cr_br.x), std::max(cr_tl.y, cr_br.y)),
        IM_COL32(40, 42, 48, 200));
    draw_list->AddLine(
        ToCanvas(plot_origin, plot_size, bounds, cx - chw, bounds.min_y),
        ToCanvas(plot_origin, plot_size, bounds, cx - chw, bounds.max_y),
        IM_COL32(210, 210, 210, 180), 1.5f);
    draw_list->AddLine(
        ToCanvas(plot_origin, plot_size, bounds, cx + chw, bounds.min_y),
        ToCanvas(plot_origin, plot_size, bounds, cx + chw, bounds.max_y),
        IM_COL32(210, 210, 210, 180), 1.5f);
    std::vector<ImVec2> cross_center;
    cross_center.push_back(ToCanvas(plot_origin, plot_size, bounds, cx, bounds.min_y));
    cross_center.push_back(ToCanvas(plot_origin, plot_size, bounds, cx, bounds.max_y));
    DrawDashedPolyline(draw_list, cross_center, IM_COL32(255, 255, 255, 120), 1.2f, 3);
  }

  draw_list->AddLine(lower_a, lower_b, IM_COL32(210, 210, 210, 255), 2.0f);
  draw_list->AddLine(upper_a, upper_b, IM_COL32(210, 210, 210, 255), 2.0f);

  {
    const double marker_step = 5.0;
    const int start_idx = static_cast<int>(std::floor(bounds.min_x / marker_step));
    const int end_idx = static_cast<int>(std::ceil(bounds.max_x / marker_step));
    for (int idx = start_idx; idx <= end_idx; ++idx) {
      const double marker_x = idx * marker_step;
      const auto lower_outer =
          ToCanvas(plot_origin, plot_size, bounds, marker_x, -rhw - 0.45);
      const auto lower_inner =
          ToCanvas(plot_origin, plot_size, bounds, marker_x, -rhw + 0.1);
      const auto upper_outer =
          ToCanvas(plot_origin, plot_size, bounds, marker_x, rhw + 0.45);
      const auto upper_inner =
          ToCanvas(plot_origin, plot_size, bounds, marker_x, rhw - 0.1);
      const bool major = (idx % 4) == 0;
      const ImU32 marker_color = major ? IM_COL32(246, 246, 246, 200)
                                       : IM_COL32(180, 180, 180, 120);
      draw_list->AddLine(lower_outer, lower_inner, marker_color, major ? 2.2f : 1.3f);
      draw_list->AddLine(upper_outer, upper_inner, marker_color, major ? 2.2f : 1.3f);
      if (major) {
        char label[32];
        std::snprintf(label, sizeof(label), "%.0fm", marker_x);
        draw_list->AddText(ImVec2(lower_outer.x - 12.0f, lower_outer.y + 4.0f),
                           IM_COL32(190, 198, 215, 210), label);
      }
    }
  }

  {
    const int n_lanes = preview.scenario_config.num_lanes;
    const double lane_w = preview.scenario_config.lane_width;
    if (n_lanes >= 2) {
      for (int i = 1; i < n_lanes; ++i) {
        const double divider_y = -rhw + i * lane_w;
        std::vector<ImVec2> lane_line;
        lane_line.push_back(ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, divider_y));
        lane_line.push_back(ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, divider_y));
        DrawDashedPolyline(draw_list, lane_line, IM_COL32(255, 255, 255, 160), 1.5f, 3);
      }
    } else {
      std::vector<ImVec2> center_line;
      center_line.push_back(ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, 0.0));
      center_line.push_back(ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, 0.0));
      DrawDashedPolyline(draw_list, center_line, IM_COL32(255, 255, 255, 180), 1.5f, 3);
    }
  }

  if (state->show_reference_line) {
    const double ref_y = preview.scenario_config.ref_origin_y;
    const auto ref_a = ToCanvas(plot_origin, plot_size, bounds, bounds.min_x, ref_y);
    const auto ref_b = ToCanvas(plot_origin, plot_size, bounds, bounds.max_x, ref_y);
    draw_list->AddLine(ref_a, ref_b, IM_COL32(242, 198, 65, 100), 1.0f);
  }

  if (state->show_obstacle && sim_mode && !obstacles.empty()) {
    const int pred_horizon = output ? output->horizon : 55;
    const int pred_step = std::max(1, pred_horizon / 18);
    for (const auto& obstacle : obstacles) {
      std::vector<ImVec2> pred_polyline;
      pred_polyline.reserve(pred_horizon / pred_step + 2);
      for (int k = 0; k <= pred_horizon; k += pred_step) {
        const double t = 0.1 * static_cast<double>(k);
        pred_polyline.push_back(ToCanvas(
            plot_origin, plot_size, bounds,
            obstacle.x + obstacle.speed_x * t,
            obstacle.y + obstacle.speed_y * t));
      }
      DrawDashedPolyline(draw_list, pred_polyline,
                         obstacle.primary ? IM_COL32(255, 140, 140, 190)
                                          : IM_COL32(255, 140, 140, 120),
                         obstacle.primary ? 2.0f : 1.2f, 2);
      for (size_t i = 1; i < pred_polyline.size(); ++i) {
        const int alpha = std::max(28, 120 - static_cast<int>(i) * 9);
        draw_list->AddCircleFilled(pred_polyline[i], obstacle.primary ? 3.0f : 2.0f,
                                   IM_COL32(255, 150, 150, alpha));
      }
    }
  }

  if (state->show_obstacle) {
    for (const auto& obstacle : obstacles) {
      DrawObstacleAt(draw_list, preview.scenario_config,
                     plot_origin, plot_size, bounds,
                     obstacle.x, obstacle.y,
                     obstacle.primary ? IM_COL32(230, 84, 84, 60)
                                      : IM_COL32(220, 92, 92, 42),
                     obstacle.primary ? IM_COL32(230, 84, 84, 255)
                                      : IM_COL32(220, 92, 92, 180),
                     obstacle.yaw);
      const double label_dx =
          preview.scenario_config.obstacle_type == ObstacleType::kCircle
              ? preview.scenario_config.obstacle_radius + 0.25
              : preview.scenario_config.obstacle_half_length + 0.25;
      const double label_dy =
          preview.scenario_config.obstacle_type == ObstacleType::kCircle
              ? preview.scenario_config.obstacle_radius + 0.12
              : preview.scenario_config.obstacle_half_width + 0.12;
      const ImVec2 id_anchor = ToCanvas(plot_origin, plot_size, bounds,
                                        obstacle.x + label_dx, obstacle.y + label_dy);
      DrawOverlayLabel(
          draw_list, ImVec2(id_anchor.x + 4.0f, id_anchor.y - 24.0f),
          "Obs" + std::to_string(obstacle.index),
          obstacle.primary ? IM_COL32(255, 232, 232, 255) : IM_COL32(245, 225, 225, 235),
          obstacle.primary ? IM_COL32(110, 24, 24, 210) : IM_COL32(90, 28, 28, 185));

      if (selected_candidate != nullptr) {
        const auto* interaction =
            FindTopologyInteraction(selected_candidate->signature, obstacle.index);
        if (interaction != nullptr) {
          DrawOverlayLabel(
              draw_list, ImVec2(id_anchor.x + 4.0f, id_anchor.y - 8.0f),
              FormatTopologyInteractionShort(*interaction),
              IM_COL32(220, 255, 228, 255), IM_COL32(14, 64, 36, 200));
        }
      }
    }
  }

  if (sim_mode && state->sim.history_x.size() >= 2) {
    std::vector<ImVec2> history_line;
    history_line.reserve(state->sim.history_x.size() + 1);
    for (size_t i = 0; i < state->sim.history_x.size(); ++i) {
      history_line.push_back(ToCanvas(plot_origin, plot_size, bounds,
                                      state->sim.history_x[i], state->sim.history_y[i]));
    }
    if (sim_display.valid && sim_display.ego_state.size() >= 2) {
      history_line.push_back(ToCanvas(plot_origin, plot_size, bounds,
                                      sim_display.ego_state(0), sim_display.ego_state(1)));
    }
    DrawPolyline(draw_list, history_line, IM_COL32(46, 204, 113, 255), 2.5f);
  }

  if (output && state->show_guidance_candidates && !output->lattice_candidates.empty()) {
    for (size_t cand_idx = 0; cand_idx < output->lattice_candidates.size(); ++cand_idx) {
      const auto& cand = output->lattice_candidates[cand_idx];
      if (cand.x.size() < 2) continue;
      std::vector<ImVec2> poly;
      const int step = std::max(1, static_cast<int>(cand.x.size()) / 30);
      for (size_t i = 0; i < cand.x.size(); i += step) {
        poly.push_back(ToCanvas(plot_origin, plot_size, bounds, cand.x[i], cand.y[i]));
      }
      if ((cand.x.size() - 1) % step != 0) {
        poly.push_back(ToCanvas(plot_origin, plot_size, bounds, cand.x.back(), cand.y.back()));
      }
      if (cand.is_best) {
        DrawPolyline(draw_list, poly, IM_COL32(0, 230, 118, 220), 2.5f);
      } else if (cand.collision_free) {
        DrawPolyline(draw_list, poly, IM_COL32(180, 130, 255, 100), 1.2f);
      } else {
        DrawDashedPolyline(draw_list, poly, IM_COL32(255, 100, 100, 60), 0.8f, 3);
      }

      if (!poly.empty() && cand_idx < output->guidance_candidates.size()) {
        char topo_label[32];
        std::snprintf(topo_label, sizeof(topo_label), "T%d",
                      output->guidance_candidates[cand_idx].topology_id);
        const ImVec2 label_pos = poly.back();
        draw_list->AddText(ImVec2(label_pos.x + 4.0f, label_pos.y - 12.0f),
                           cand.is_best ? IM_COL32(0, 230, 118, 235)
                                        : IM_COL32(200, 170, 255, 190),
                           topo_label);
      }
    }
  }

  if (output && state->show_dp_lattice_nodes && !output->dp_lattice_nodes.empty()) {
    static constexpr ImU32 kLayerColors[] = {
        IM_COL32(255, 180, 60, 180),
        IM_COL32(60, 200, 255, 180),
        IM_COL32(200, 100, 255, 180),
        IM_COL32(100, 255, 160, 180),
        IM_COL32(255, 120, 120, 180),
        IM_COL32(255, 220, 100, 180),
        IM_COL32(120, 180, 255, 180),
        IM_COL32(220, 160, 220, 180),
    };
    constexpr int kNumLayerColors = sizeof(kLayerColors) / sizeof(kLayerColors[0]);

    ImVec2 prev_best_pos = {-1.0f, -1.0f};
    int prev_best_layer = -1;

    for (const auto& node : output->dp_lattice_nodes) {
      const ImVec2 pos = ToCanvas(plot_origin, plot_size, bounds, node.x, node.y);
      const ImU32 layer_color = kLayerColors[node.layer % kNumLayerColors];

      if (node.on_best_path) {
        draw_list->AddCircleFilled(pos, 6.0f, IM_COL32(0, 230, 118, 240));
        draw_list->AddCircle(pos, 7.0f, IM_COL32(255, 255, 255, 200), 0, 1.5f);
        if (prev_best_layer >= 0 && node.layer == prev_best_layer + 1) {
          draw_list->AddLine(prev_best_pos, pos, IM_COL32(0, 230, 118, 160), 2.0f);
        }
        prev_best_pos = pos;
        prev_best_layer = node.layer;
      } else if (node.reachable) {
        draw_list->AddCircleFilled(pos, 3.5f, layer_color);
      } else {
        draw_list->AddCircle(pos, 3.0f, IM_COL32(120, 120, 120, 100), 0, 1.0f);
      }
    }
  }

  if (output && state->show_selected_guidance && output->guidance_used &&
      output->guidance_x.size() >= 2) {
    std::vector<ImVec2> guidance_polyline;
    guidance_polyline.reserve(output->guidance_x.size());
    for (size_t i = 0; i < output->guidance_x.size(); ++i) {
      guidance_polyline.push_back(
          ToCanvas(plot_origin, plot_size, bounds, output->guidance_x[i], output->guidance_y[i]));
    }
    DrawDashedPolyline(draw_list, guidance_polyline, IM_COL32(0, 230, 118, 220), 2.4f, 2);
  }

  if (output && state->show_selected_guidance &&
      output->guidance_used && !output->guidance_x.empty()) {
    const auto label_pos = ToCanvas(plot_origin, plot_size, bounds,
                                    output->guidance_x.back(), output->guidance_y.back());
    draw_list->AddCircleFilled(label_pos, 3.5f, IM_COL32(0, 230, 118, 230));
    char topo_label[64];
    std::snprintf(topo_label, sizeof(topo_label), "topo %d",
                  output->selected_guidance_topology_id);
    draw_list->AddText(ImVec2(label_pos.x + 8.0f, label_pos.y - 18.0f),
                       IM_COL32(0, 230, 118, 230), topo_label);
  }

  if (state->show_initial_guess) {
    const std::vector<double>& init_x = (output && output->initial_x.size() >= 2)
                                            ? output->initial_x : preview.x;
    const std::vector<double>& init_y = (output && output->initial_y.size() >= 2)
                                            ? output->initial_y : preview.y;
    if (init_x.size() >= 2) {
      std::vector<ImVec2> initial_polyline;
      initial_polyline.reserve(init_x.size());
      for (size_t i = 0; i < init_x.size(); ++i) {
        initial_polyline.push_back(ToCanvas(plot_origin, plot_size, bounds, init_x[i], init_y[i]));
      }
      DrawDashedPolyline(draw_list, initial_polyline, IM_COL32(160, 160, 160, 220), 2.0f, 2);
      draw_list->AddCircleFilled(initial_polyline.back(), 3.5f, IM_COL32(160, 160, 160, 220));
    }
  }

  if (output && state->show_pp_seed && output->pp_seed_x.size() >= 2) {
    std::vector<ImVec2> pp_seed_polyline;
    pp_seed_polyline.reserve(output->pp_seed_x.size());
    for (size_t i = 0; i < output->pp_seed_x.size(); ++i) {
      pp_seed_polyline.push_back(
          ToCanvas(plot_origin, plot_size, bounds, output->pp_seed_x[i], output->pp_seed_y[i]));
    }
    DrawDashedPolyline(draw_list, pp_seed_polyline, IM_COL32(255, 176, 59, 220), 2.0f, 2);
    draw_list->AddCircleFilled(pp_seed_polyline.back(), 3.5f, IM_COL32(255, 176, 59, 220));
  }

  if (output && state->show_optimized_trajectory && displayed_plan.x.size() >= 2) {
    DrawTrajectoryRibbon(draw_list, output->vehicle_config,
                         plot_origin, plot_size, bounds,
                         displayed_plan.x, displayed_plan.y, displayed_plan.speed,
                         state->vehicle_config.model.min_speed,
                         state->vehicle_config.model.max_speed);

    const VehicleCircleApproximation geometry(output->vehicle_config.body);
    if (state->show_collision_circles &&
        output->scenario_config.obstacle_type != ObstacleType::kPolygonExact) {
      const int sample_step = std::max(1, static_cast<int>(displayed_plan.x.size() / 9));
      for (size_t i = 0; i < displayed_plan.x.size(); i += static_cast<size_t>(sample_step)) {
        Vector vehicle_state(4);
        vehicle_state << displayed_plan.x[i], displayed_plan.y[i], displayed_plan.yaw[i],
            displayed_plan.speed[i];
        const auto circle_centers = geometry.EvaluateWorldCenters(vehicle_state);
        for (int circle_index = 0; circle_index < geometry.CircleCount(); ++circle_index) {
          const auto circle = MakeObstaclePolyline(plot_origin, plot_size, bounds,
                                                   circle_centers[circle_index](0),
                                                   circle_centers[circle_index](1),
                                                   geometry.Circles()[circle_index].radius, 24);
          DrawPolyline(draw_list, circle, IM_COL32(46, 204, 113, 96), 1.1f);
        }
      }
    }
  }

  if (!sim_mode && !preview.x.empty()) {
    const auto start_pt = ToCanvas(plot_origin, plot_size, bounds, preview.x.front(), preview.y.front());
    draw_list->AddCircleFilled(start_pt, 4.0f, IM_COL32(255, 255, 255, 255));
  }
  if (!sim_mode && state->show_optimized_trajectory && output && !output->x.empty()) {
    const auto end_pt = ToCanvas(plot_origin, plot_size, bounds, output->x.back(), output->y.back());
    draw_list->AddCircleFilled(end_pt, 4.0f, IM_COL32(72, 149, 239, 255));
  }
  if (sim_mode && sim_display.valid && sim_display.ego_state.size() >= 3) {
    if (state->show_vehicle_outline) {
      const auto ego_outline = MakeVehicleOutline(
          state->vehicle_config, plot_origin, plot_size, bounds,
          sim_display.ego_state(0), sim_display.ego_state(1), sim_display.ego_state(2));
      FillClosedPolygon(draw_list, ego_outline, IM_COL32(255, 200, 40, 80));
      DrawPolyline(draw_list, ego_outline, IM_COL32(255, 200, 40, 200), 1.5f);
    }
    const auto ego_pt = ToCanvas(plot_origin, plot_size, bounds,
                                 sim_display.ego_state(0), sim_display.ego_state(1));
    draw_list->AddCircleFilled(ego_pt, 5.0f, IM_COL32(255, 200, 40, 255));
    draw_list->AddText(ImVec2(ego_pt.x + 6.0f, ego_pt.y - 14.0f),
                       IM_COL32(255, 200, 40, 200), "RA");
  }

  draw_list->AddText(ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 10.0f),
                     IM_COL32(235, 235, 235, 255),
                     sim_mode ? "Simulation"
                              : "Reference / initial guess / PP seed / init controls / optimized trajectory");
  char view_label[96];
  std::snprintf(view_label, sizeof(view_label), "zoom %.2fx  RMB drag to pan", state->scene_zoom);
  draw_list->AddText(ImVec2(canvas_max.x - 180.0f, canvas_pos.y + 10.0f),
                     IM_COL32(190, 198, 215, 255), view_label);
  const bool has_guidance_bundle = output && !output->lattice_candidates.empty();
  const bool has_pp_seed = output && !output->pp_seed_x.empty();
  const bool has_selected_guidance = output && output->guidance_used && !output->guidance_x.empty();
  const bool has_dp_nodes = output && !output->dp_lattice_nodes.empty();
  const float legend_height =
      (sim_mode ? 146.0f : 104.0f) +
      (has_guidance_bundle ? 18.0f : 0.0f) +
      (has_pp_seed ? 18.0f : 0.0f) +
      (has_selected_guidance ? 18.0f : 0.0f) +
      (has_dp_nodes ? 18.0f : 0.0f);
  const ImVec2 legend_anchor(canvas_pos.x + 14.0f, canvas_max.y - legend_height);
  draw_list->AddRectFilled(ImVec2(legend_anchor.x - 6.0f, legend_anchor.y - 6.0f),
                           ImVec2(legend_anchor.x + 170.0f, legend_anchor.y + legend_height - 12.0f),
                           IM_COL32(10, 12, 18, 170), 6.0f);
  float y_off = 0.0f;
  if (!sim_mode) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(242, 198, 65, 220), "Reference", state->show_reference_line);
    y_off += 18.0f;
  }
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                  IM_COL32(230, 84, 84, 255), "Obstacle", state->show_obstacle);
  y_off += 18.0f;
  if (!sim_mode) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(160, 160, 160, 220), "Init Controls", state->show_initial_guess, true);
    y_off += 18.0f;
  }
  if (has_pp_seed) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(255, 176, 59, 220), "PP Seed", state->show_pp_seed, true);
    y_off += 18.0f;
  }
  if (has_guidance_bundle) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(180, 130, 255, 200),
                    "Guidance candidates",
                    state->show_guidance_candidates);
    y_off += 18.0f;
  }
  if (has_dp_nodes) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(255, 180, 60, 200),
                    "DP lattice nodes",
                    state->show_dp_lattice_nodes);
    y_off += 18.0f;
  }
  if (has_selected_guidance) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(0, 230, 118, 220),
                    "Initial Guess",
                    state->show_selected_guidance,
                    true);
    y_off += 18.0f;
  }
  DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                  IM_COL32(72, 149, 239, 255), sim_mode ? "Current plan" : "Opt", state->show_optimized_trajectory);
  y_off += 18.0f;
  if (sim_mode) {
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(46, 204, 113, 255), "History", true);
    y_off += 18.0f;
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(255, 200, 40, 255), "Ego", true);
    y_off += 18.0f;
    DrawLegendEntry(draw_list, ImVec2(legend_anchor.x, legend_anchor.y + y_off),
                    IM_COL32(230, 84, 84, 120), "Obs prediction", true);
  }

  if (output && output->guidance_used) {
    const float box_width = 320.0f;
    const float box_height = 122.0f;
    const ImVec2 info_anchor(canvas_max.x - box_width - 16.0f, canvas_max.y - box_height - 16.0f);
    draw_list->AddRectFilled(info_anchor,
                             ImVec2(info_anchor.x + box_width, info_anchor.y + box_height),
                             IM_COL32(10, 12, 18, 185), 6.0f);
    char line1[96];
      std::snprintf(line1, sizeof(line1), "Initial Guess  T%d  cand=%d",
                    output->selected_guidance_topology_id,
                    static_cast<int>(output->guidance_candidates.size()));
    draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 8.0f),
                       IM_COL32(235, 235, 235, 255), line1);
    if (selected_candidate != nullptr) {
      const std::string selected_relations =
          FormatTopologySignatureReadable(selected_candidate->signature);
      char line2[128];
      std::snprintf(line2, sizeof(line2), "cost=%.2f  %s",
                    selected_candidate->cost,
                    selected_candidate->collision_free ? "safe" : "collision");
      draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 28.0f),
                         IM_COL32(190, 198, 215, 255), line2);
      draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 46.0f),
                         IM_COL32(160, 220, 180, 235),
                         "Selected relations");
      draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 64.0f),
                         IM_COL32(160, 220, 180, 235),
                         selected_relations.c_str());
      char line3[128];
      std::snprintf(line3, sizeof(line3),
                    "guide %.1f  track %.1f  solver %.1f ms",
                    output->guidance_time_ms, output->tracking_time_ms,
                    output->solver_time_ms);
      draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 84.0f),
                         IM_COL32(190, 198, 215, 255), line3);
      draw_list->AddText(ImVec2(info_anchor.x + 10.0f, info_anchor.y + 102.0f),
                         IM_COL32(154, 162, 180, 235),
                         "Legend: R=+d, L=-d, A=After, B=Before");
    }
  }

  ImGui::Dummy(canvas_size);
}

void DrawSolverLogPanel(const FrontendState& state) {
  if (!state.has_result) {
    ImGui::TextWrapped("No solver log yet.");
    return;
  }

  const PlanningOutput& output = state.result;
  ImGui::Text("Total: %.1f ms | Guidance: %.1f | Tracking: %.1f | Solver: %.1f",
              output.planning_time_ms, output.guidance_time_ms,
              output.tracking_time_ms, output.solver_time_ms);
  ImGui::Text("Converged: %s", output.converged ? "true" : "false");
  ImGui::Text("Horizon: %d", output.horizon);
  ImGui::Text("Outer iterations: %d", output.outer_iterations);
  ImGui::Text("Total inner iterations: %d", output.total_inner_iterations);
  ImGui::Text("Initial cost: %.4f", output.initial_cost);
  ImGui::Text("Final cost: %.4f", output.final_cost);
  ImGui::Text("Initial violation: %.6f", output.initial_violation);
  ImGui::Text("Final violation: %.6f", output.final_violation);
  ImGui::Text("Guidance used: %s", output.guidance_used ? "true" : "false");
  ImGui::Text("Selected topology: %d", output.selected_guidance_topology_id);
  ImGui::Text("Guidance candidates: %d", static_cast<int>(output.guidance_candidates.size()));

  ImGui::SeparatorText("Outer Loop");
  ImGui::BeginChild("outer_logs", ImVec2(0.0f, 180.0f), true);
  for (const auto& log : output.outer_logs) {
    ImGui::Text("it=%d inner=%d base=%.3f aug=%.3f viol=%.5f best=%.5f pen=%.1f %s",
                log.outer_iteration,
                log.inner_iterations,
                log.base_cost,
                log.augmented_cost,
                log.max_violation,
                log.best_violation_so_far,
                log.max_penalty,
                log.penalty_updated ? "penalty++" : "");
  }
  ImGui::EndChild();
}

void DrawGuidancePanel(const FrontendState& state) {
  if (!state.has_result) {
    ImGui::TextWrapped("No guidance result yet.");
    return;
  }

  const PlanningOutput& output = state.result;
  ImGui::Text("Guidance used: %s", output.guidance_used ? "true" : "false");
  ImGui::Text("Selected topology: %d", output.selected_guidance_topology_id);
  ImGui::Text("Candidates: %d", static_cast<int>(output.guidance_candidates.size()));

  if (output.guidance_memory.valid) {
    ImGui::Text("Memory selected topology: %d", output.guidance_memory.selected_topology_id);
    ImGui::TextWrapped("Memory relations: %s",
                       FormatSignatureKeyReadable(
                           output.guidance_memory.selected_signature_key).c_str());
  }

  DrawRelationLegendTable();

  ImGui::SeparatorText("Candidate Bundle");
  ImGui::BeginChild("guidance_candidates", ImVec2(0.0f, 160.0f), true);
  if (output.guidance_candidates.empty()) {
    ImGui::TextWrapped("No guidance candidates.");
  } else {
    for (const auto& candidate : output.guidance_candidates) {
      ImGui::Text("topo=%d cost=%.3f %s %s",
                  candidate.topology_id,
                  candidate.cost,
                  candidate.selected ? "[selected]" : "",
                  candidate.collision_free ? "safe" : "collision");
      ImGui::TextWrapped("relations: %s",
                         FormatTopologySignatureReadable(candidate.signature).c_str());
      ImGui::Separator();
    }
  }
  ImGui::EndChild();
}

ScenePreview BuildPreview(const VehicleBicycleConfig& vehicle_config,
                          const StaticObstacleScenarioConfig& base_config,
                          const std::vector<FrontendObstacle>& obstacles) {
  ScenePreview preview;
  preview.vehicle_config = vehicle_config;
  preview.scenario_config =
      ScenarioConfigForObstacleField(base_config, obstacles, base_config.initial_x);
  try {
    auto prepared = PreparePlanningScenario(
        vehicle_config, preview.scenario_config, obstacles, base_config.initial_x);
    auto& scenario = prepared.scenario;
    const auto trajectory =
        scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
    preview.x.resize(trajectory.Horizon() + 1);
    preview.y.resize(trajectory.Horizon() + 1);
    preview.yaw.resize(trajectory.Horizon() + 1);
    preview.speed.resize(trajectory.Horizon() + 1);
    for (int k = 0; k <= trajectory.Horizon(); ++k) {
      preview.x[k] = trajectory.State(k)(0);
      preview.y[k] = trajectory.State(k)(1);
      preview.yaw[k] = trajectory.State(k)(2);
      preview.speed[k] = trajectory.State(k)(3);
    }
    preview.valid = true;
    preview.status = "Preview updated";
  } catch (const std::exception& e) {
    preview.status = e.what();
  }
  return preview;
}

void RefreshPreviewIfNeeded(FrontendState* state) {
  if (!state || !state->preview_dirty) {
    return;
  }
  state->preview = BuildPreview(
      state->vehicle_config, state->scenario_config, state->obstacles);
  state->preview_dirty = false;
}

PlanningOutput RunPlanningJob(const std::filesystem::path& config_path,
                              const VehicleBicycleConfig& vehicle_config,
                              const StaticObstacleScenarioConfig& base_config,
                              const std::vector<FrontendObstacle>& obstacles) {
  PlanningOutput output;
  output.vehicle_config = vehicle_config;
  output.scenario_config =
      ScenarioConfigForObstacleField(base_config, obstacles, base_config.initial_x);
  try {
    const auto t_start = std::chrono::steady_clock::now();
    auto prepared = PreparePlanningScenario(
        vehicle_config, output.scenario_config, obstacles, base_config.initial_x);
    auto& scenario = prepared.scenario;
    const auto t_solver_start = std::chrono::steady_clock::now();
    const auto solve_result = SolvePreparedScenarioRobustly(
        prepared, scenario.initial_state, scenario.initial_controls);
    const auto t_solver_end = std::chrono::steady_clock::now();
    const auto t_end = t_solver_end;
    const auto initial_trajectory = scenario.base_problem->Rollout(
        scenario.initial_state, scenario.initial_controls);
    const auto& result = *solve_result.result;

    std::filesystem::create_directories("build");
    output.trajectory_csv_path = "build/imgui_frontend_trajectory.csv";
    my_al_ilqr::WriteTrajectoryCsv(output.trajectory_csv_path, result.trajectory);

    const int horizon = result.trajectory.Horizon();
    output.horizon = horizon;
    output.outer_logs = result.outer_logs;
    output.outer_iterations = static_cast<int>(result.outer_logs.size());
    output.total_inner_iterations = 0;
    for (const auto& log : result.outer_logs) {
      output.total_inner_iterations += log.inner_iterations;
      output.base_cost_history.push_back(log.base_cost);
      output.violation_history.push_back(log.max_violation);
      output.penalty_history.push_back(log.max_penalty);
    }
    output.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    output.guidance_time_ms = prepared.guidance_time_ms;
    output.tracking_time_ms = prepared.tracking_time_ms;
    output.solver_time_ms =
        std::chrono::duration<double, std::milli>(t_solver_end - t_solver_start).count();
    output.initial_cost = scenario.base_problem->TotalCost(initial_trajectory);
    output.final_cost = scenario.base_problem->TotalCost(result.trajectory);
    output.initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);
    output.final_violation = scenario.constrained_problem->MaxViolation(result.trajectory);
    output.converged = result.converged;
    output.x.resize(horizon + 1);
    output.y.resize(horizon + 1);
    output.initial_x.resize(horizon + 1);
    output.initial_y.resize(horizon + 1);
    output.pp_seed_x = prepared.pp_seed_x;
    output.pp_seed_y = prepared.pp_seed_y;
    output.guidance_x.resize(prepared.guidance.selected_guidance.size());
    output.guidance_y.resize(prepared.guidance.selected_guidance.size());
    output.yaw.resize(horizon + 1);
    output.speed.resize(horizon + 1);
    output.accel.resize(horizon);
    output.kappa.resize(horizon);

    for (int k = 0; k <= horizon; ++k) {
      output.initial_x[k] = initial_trajectory.State(k)(0);
      output.initial_y[k] = initial_trajectory.State(k)(1);
      output.x[k] = result.trajectory.State(k)(0);
      output.y[k] = result.trajectory.State(k)(1);
      output.yaw[k] = result.trajectory.State(k)(2);
      output.speed[k] = result.trajectory.State(k)(3);
      if (k < horizon) {
        output.accel[k] = result.trajectory.Control(k)(0);
        output.kappa[k] = std::tan(result.trajectory.Control(k)(1)) / vehicle_config.model.wheelbase;
      }
    }
    for (size_t k = 0; k < prepared.guidance.selected_guidance.size(); ++k) {
      output.guidance_x[k] = prepared.guidance.selected_guidance[k].x;
      output.guidance_y[k] = prepared.guidance.selected_guidance[k].y;
    }

    std::ostringstream terminal_state;
    terminal_state << result.trajectory.State(horizon).transpose();

    output.success = true;
    output.terminal_state = terminal_state.str();
    output.best_violation = result.best_violation;
    output.guidance_used = !prepared.guidance.selected_guidance.empty();
    output.guidance_memory = prepared.guidance.planner_memory;
    output.selected_guidance_topology_id = prepared.guidance.selected_topology_id;
    output.guidance_candidates = std::move(prepared.guidance.candidate_bundle);
    output.lattice_candidates = std::move(scenario.lattice_candidates);
    output.dp_lattice_nodes = std::move(scenario.dp_lattice_nodes);
    std::ostringstream status;
    status << "Config: " << config_path.string() << "\n"
           << "Planning time: " << output.planning_time_ms << " ms\n"
           << "Guidance time: " << output.guidance_time_ms << " ms\n"
           << "Tracking time: " << output.tracking_time_ms << " ms\n"
           << "AL-iLQR time: " << output.solver_time_ms << " ms\n"
           << "Init controls: " << prepared.selected_init_controls_label << "\n"
           << "Outer iterations: " << output.outer_iterations
           << ", total inner iterations: " << output.total_inner_iterations << "\n"
           << "Initial cost -> final cost: " << output.initial_cost
           << " -> " << output.final_cost << "\n"
           << "Initial violation -> final violation: " << output.initial_violation
           << " -> " << output.final_violation << "\n"
           << "Best violation: " << result.best_violation << "\n"
           << "Guidance topology: " << output.selected_guidance_topology_id << "\n"
           << "Trajectory CSV: " << output.trajectory_csv_path.string() << "\n"
           << "Pure ImGui frontend: visualization stays in the main window.";
    output.status = status.str();
  } catch (const std::exception& e) {
    output.status = e.what();
  }
  return output;
}

std::vector<Vector> ShiftControls(const std::vector<Vector>& controls,
                                  int shift) {
  const int n = static_cast<int>(controls.size());
  if (n == 0 || shift <= 0) return controls;
  std::vector<Vector> shifted;
  shifted.reserve(n);
  for (int i = shift; i < n; ++i) {
    shifted.push_back(controls[i]);
  }
  while (static_cast<int>(shifted.size()) < n) {
    shifted.push_back(shifted.empty() ? Vector::Zero(2) : shifted.back());
  }
  return shifted;
}

void ClearSimulationWarmStart(SimulationState* sim) {
  if (!sim) {
    return;
  }
  sim->warm_start_valid = false;
  sim->previous_controls.clear();
}

void ClearSimulationGuidanceMemory(SimulationState* sim) {
  if (!sim) {
    return;
  }
  sim->guidance_memory = my_al_ilqr::GlobalGuidanceMemory{};
}

void ClearSimulationPlanningRequest(FrontendState* state) {
  if (!state) {
    return;
  }
  state->planning_request = SimulationPlanningRequest{};
  state->planning_request_ready = false;
}

void TrimAppliedControlHistory(SimulationState* sim) {
  if (!sim) {
    return;
  }
  const size_t sample_count = std::min(
      sim->applied_control_time_history.size(),
      std::min(sim->applied_accel_history.size(), sim->applied_steer_history.size()));
  if (sample_count == 0) {
    sim->applied_control_time_history.clear();
    sim->applied_accel_history.clear();
    sim->applied_steer_history.clear();
    return;
  }

  size_t trim_count = 0;
  const double cutoff_time =
      std::max(0.0, sim->applied_control_time_history[sample_count - 1] - kAppliedControlPlotWindowSec);
  while (trim_count < sample_count &&
         sim->applied_control_time_history[trim_count] < cutoff_time) {
    ++trim_count;
  }
  if (sample_count - trim_count > static_cast<size_t>(kAppliedControlHistoryMaxSamples)) {
    trim_count = sample_count - static_cast<size_t>(kAppliedControlHistoryMaxSamples);
  }
  if (trim_count == 0) {
    return;
  }

  sim->applied_control_time_history.erase(
      sim->applied_control_time_history.begin(),
      sim->applied_control_time_history.begin() + static_cast<std::ptrdiff_t>(trim_count));
  sim->applied_accel_history.erase(
      sim->applied_accel_history.begin(),
      sim->applied_accel_history.begin() + static_cast<std::ptrdiff_t>(trim_count));
  sim->applied_steer_history.erase(
      sim->applied_steer_history.begin(),
      sim->applied_steer_history.begin() + static_cast<std::ptrdiff_t>(trim_count));
}

std::vector<Vector> ExtractSimulationPlanControls(const PlanningOutput& output,
                                                  const VehicleBicycleConfig& vehicle_config,
                                                  int start_index) {
  std::vector<Vector> controls;
  if (output.horizon <= 0 || output.accel.empty() || output.kappa.empty()) {
    return controls;
  }
  start_index = std::clamp(start_index, 0, output.horizon);
  controls.reserve(std::max(0, output.horizon - start_index));
  for (int k = start_index; k < output.horizon; ++k) {
    Vector ctrl(2);
    ctrl << output.accel[k],
        std::atan(output.kappa[k] * vehicle_config.model.wheelbase);
    controls.push_back(ctrl);
  }
  return controls;
}

void EnsureSimulationVectors(SimulationState* sim) {
  if (!sim) {
    return;
  }
  if (sim->ego_state.size() != 4) {
    sim->ego_state = Vector::Zero(4);
  }
  if (sim->applied_control.size() != 2) {
    sim->applied_control = Vector::Zero(2);
  }
}

bool HasCurrentSimulationControl(const SimulationState& sim) {
  return sim.applied_control.size() == 2 && sim.applied_control_valid;
}

void RebuildSimulationRollout(SimulationState* sim,
                              const VehicleBicycleConfig& vehicle_config) {
  if (!sim) {
    return;
  }
  EnsureSimulationVectors(sim);
  const my_al_ilqr::KinematicBicycleModel dynamics(vehicle_config.model.wheelbase);

  sim->anim_x.clear();
  sim->anim_y.clear();
  sim->anim_yaw.clear();
  sim->anim_speed.clear();
  sim->anim_accel.clear();
  sim->anim_steer.clear();

  sim->anim_x.push_back(sim->ego_state(0));
  sim->anim_y.push_back(sim->ego_state(1));
  sim->anim_yaw.push_back(sim->ego_state(2));
  sim->anim_speed.push_back(sim->ego_state(3));

  Vector rollout_state = sim->ego_state;
  const bool include_current_control = HasCurrentSimulationControl(*sim);
  const double remaining_current_dt =
      std::clamp(kTrajectoryStepDt - sim->control_phase_time, 0.0, kTrajectoryStepDt);
  if (include_current_control && remaining_current_dt > 1e-6) {
    rollout_state = dynamics.NextState(rollout_state, sim->applied_control, remaining_current_dt);
    sim->anim_x.push_back(rollout_state(0));
    sim->anim_y.push_back(rollout_state(1));
    sim->anim_yaw.push_back(rollout_state(2));
    sim->anim_speed.push_back(rollout_state(3));
    sim->anim_accel.push_back(sim->applied_control(0));
    sim->anim_steer.push_back(sim->applied_control(1));
  }

  for (const Vector& control : sim->executing_controls) {
    rollout_state = dynamics.NextState(rollout_state, control, kTrajectoryStepDt);
    sim->anim_x.push_back(rollout_state(0));
    sim->anim_y.push_back(rollout_state(1));
    sim->anim_yaw.push_back(rollout_state(2));
    sim->anim_speed.push_back(rollout_state(3));
    sim->anim_accel.push_back(control(0));
    sim->anim_steer.push_back(control(1));
  }

  sim->anim_idx = 0;
  sim->anim_total = static_cast<int>(sim->anim_accel.size());
  sim->animating = sim->anim_x.size() >= 2;
  sim->anim_last_time = 0.0;
}

void ConsumeSimulationRolloutPrefix(SimulationState* sim) {
  if (!sim) {
    return;
  }
  if (sim->anim_x.size() >= 2 &&
      sim->anim_y.size() == sim->anim_x.size() &&
      sim->anim_yaw.size() == sim->anim_x.size() &&
      sim->anim_speed.size() == sim->anim_x.size()) {
    sim->anim_x.erase(sim->anim_x.begin());
    sim->anim_y.erase(sim->anim_y.begin());
    sim->anim_yaw.erase(sim->anim_yaw.begin());
    sim->anim_speed.erase(sim->anim_speed.begin());
  }
  if (!sim->anim_accel.empty()) {
    sim->anim_accel.erase(sim->anim_accel.begin());
  }
  if (!sim->anim_steer.empty()) {
    sim->anim_steer.erase(sim->anim_steer.begin());
  }
  sim->anim_idx = 0;
  sim->anim_total = static_cast<int>(sim->anim_accel.size());
  sim->animating = sim->anim_x.size() >= 2;
}

void InstallSimulationAnimation(SimulationState* sim,
                                const VehicleBicycleConfig& vehicle_config,
                                const PlanningOutput& output) {
  if (!sim) {
    return;
  }
  EnsureSimulationVectors(sim);

  int elapsed_steps =
      output.simulation_step_at_request >= 0
          ? std::max(0, sim->step_count - output.simulation_step_at_request)
          : 0;

  if (elapsed_steps > 0 &&
      static_cast<int>(output.x.size()) > elapsed_steps &&
      static_cast<int>(output.y.size()) > elapsed_steps) {
    const double ego_x = sim->ego_state(0);
    const double ego_y = sim->ego_state(1);
    int best_step = elapsed_steps;
    double best_dist_sq = std::numeric_limits<double>::max();
    const int lo = std::max(0, elapsed_steps - 3);
    const int hi = std::min(static_cast<int>(output.x.size()) - 1,
                            elapsed_steps + 3);
    for (int k = lo; k <= hi; ++k) {
      const double dx = ego_x - output.x[k];
      const double dy = ego_y - output.y[k];
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_dist_sq) {
        best_dist_sq = d2;
        best_step = k;
      }
    }
    elapsed_steps = std::min(best_step, output.horizon);
  }

  const bool mid_interval = sim->control_phase_time > 1e-6;
  std::vector<Vector> plan_controls =
      ExtractSimulationPlanControls(output, vehicle_config, elapsed_steps);

  if (!mid_interval) {
    if (!plan_controls.empty()) {
      sim->applied_control = plan_controls.front();
      sim->applied_control_valid = true;
      sim->executing_controls.assign(plan_controls.begin() + 1, plan_controls.end());
    } else {
      sim->applied_control = Vector::Zero(2);
      sim->applied_control_valid = false;
      sim->executing_controls.clear();
    }
  } else {
    const int future_start_index = std::min(
        output.horizon, elapsed_steps + 1);
    sim->executing_controls =
        ExtractSimulationPlanControls(output, vehicle_config, future_start_index);
  }
  sim->step_pending = false;
  sim->steps_since_replan = 0;
  sim->last_applied_serial = output.request_serial;
  sim->accepted_plan_count += 1;
  sim->last_planning_time_ms = output.planning_time_ms;
  sim->last_guidance_time_ms = output.guidance_time_ms;
  sim->last_tracking_time_ms = output.tracking_time_ms;
  sim->last_solver_time_ms = output.solver_time_ms;
  sim->last_planning_deadline_ms = output.deadline_ms;
  RebuildSimulationRollout(sim, vehicle_config);
}

void SyncSimulationSnapshotFromRuntime(FrontendState* state) {
  if (!state) {
    return;
  }
  std::lock_guard<std::mutex> lock(state->simulation_mutex);
  state->sim = state->sim_runtime;
  state->obstacles = state->obstacles_runtime;
}

void SyncSimulationRuntimeFromSnapshot(FrontendState* state) {
  if (!state) {
    return;
  }
  std::lock_guard<std::mutex> lock(state->simulation_mutex);
  state->vehicle_config_runtime = state->vehicle_config;
  state->scenario_config_runtime = state->scenario_config;
  state->sim_runtime = state->sim;
  state->obstacles_runtime = state->obstacles;
}

void MaybeQueueSimulationPlanningRequestLocked(FrontendState* state) {
  if (!state || state->planning_request_in_flight || state->planning_request_ready) {
    return;
  }

  SimulationState& sim = state->sim_runtime;
  if (!sim.step_pending) {
    return;
  }

  SimulationPlanningRequest request;
  request.valid = true;
  request.vehicle_config = state->vehicle_config_runtime;
  request.scenario_config = state->scenario_config_runtime;
  request.obstacles = state->obstacles_runtime;
  request.ego_state = sim.ego_state;
  request.use_warm_start = sim.use_warm_start;
  request.guidance_memory = sim.guidance_memory;
  request.request_serial = sim.next_request_serial++;
  request.step_count = sim.step_count;
  request.sim_time = sim.sim_time;
  request.deadline_ms =
      1000.0 * static_cast<double>(sim.replan_steps) * kTrajectoryStepDt;

  if (request.use_warm_start && sim.warm_start_valid &&
      !sim.previous_controls.empty()) {
    request.warm_start_controls =
        ShiftControls(sim.previous_controls, std::max(0, sim.steps_since_replan));
  }

  sim.last_requested_serial = request.request_serial;
  sim.step_pending = false;
  state->planning_request = std::move(request);
  state->planning_request_ready = true;
}

void PhysicsTickSimulation(SimulationState* sim,
                           std::vector<FrontendObstacle>* obstacles,
                           const VehicleBicycleConfig& vehicle_config,
                           const StaticObstacleScenarioConfig& scenario_config,
                           double dt) {
  if (!sim || !obstacles) {
    return;
  }
  EnsureSimulationVectors(sim);

  const bool run_requested = sim->running || sim->manual_steps_remaining > 0;
  const bool has_command =
      !sim->executing_controls.empty() || sim->applied_control_valid;
  if (!run_requested) {
    return;
  }
  if (!has_command) {
    sim->step_pending = true;
    return;
  }

  const my_al_ilqr::KinematicBicycleModel dynamics(vehicle_config.model.wheelbase);
  double remaining_dt = dt;
  bool rollout_consumed = false;
  while (remaining_dt > 1e-9) {
    const double time_to_boundary =
        std::max(1e-9, kTrajectoryStepDt - sim->control_phase_time);
    const double substep_dt = std::min(remaining_dt, time_to_boundary);

    sim->history_x.push_back(sim->ego_state(0));
    sim->history_y.push_back(sim->ego_state(1));
    AdvanceObstacleWorld(scenario_config, sim->ego_state(0), substep_dt, obstacles);
    sim->ego_state =
        dynamics.NextState(sim->ego_state, sim->applied_control, substep_dt);
    sim->sim_time += substep_dt;
    sim->control_phase_time += substep_dt;
    remaining_dt -= substep_dt;

    if (sim->control_phase_time < kTrajectoryStepDt - 1e-9) {
      continue;
    }

    sim->control_phase_time = 0.0;
    ConsumeSimulationRolloutPrefix(sim);
    rollout_consumed = true;
    sim->step_count += 1;
    sim->steps_since_replan += 1;
    if (sim->manual_steps_remaining > 0) {
      sim->manual_steps_remaining -= 1;
    }
    if (sim->steps_since_replan >= sim->replan_steps) {
      sim->step_pending = true;
    }
    if (!sim->executing_controls.empty()) {
      sim->applied_control = sim->executing_controls.front();
      sim->applied_control_valid = true;
      sim->executing_controls.erase(sim->executing_controls.begin());
    } else if (sim->running) {
      sim->applied_control = Vector::Zero(2);
      sim->applied_control_valid = false;
      sim->step_pending = true;
      break;
    }
  }

  const double next_history_time =
      sim->applied_control_time_history.empty() ? -std::numeric_limits<double>::infinity()
                                                : sim->applied_control_time_history.back();
  if (sim->sim_time - next_history_time >= kTrajectoryStepDt - 1e-6) {
    sim->applied_control_time_history.push_back(sim->sim_time);
    sim->applied_accel_history.push_back(sim->applied_control(0));
    sim->applied_steer_history.push_back(sim->applied_control(1));
    TrimAppliedControlHistory(sim);
  }

  if (sim->running && sim->executing_controls.empty()) {
    sim->step_pending = true;
  }
  if ((!rollout_consumed && !sim->executing_controls.empty() && !sim->animating) ||
      (!rollout_consumed && sim->step_pending && sim->anim_x.size() < 2)) {
    RebuildSimulationRollout(sim, vehicle_config);
  }
}

void RunPhysicsThread(FrontendState* state) {
  if (!state) {
    return;
  }
  auto next_tick = std::chrono::steady_clock::now();
  while (!state->physics_shutdown.load()) {
    double sleep_dt = 0.01;
    {
      std::lock_guard<std::mutex> lock(state->simulation_mutex);
      sleep_dt = std::clamp(state->sim_runtime.physics_tick_dt, 0.005, kTrajectoryStepDt);
      PhysicsTickSimulation(&state->sim_runtime, &state->obstacles_runtime,
                            state->vehicle_config_runtime, state->scenario_config_runtime, sleep_dt);
      MaybeQueueSimulationPlanningRequestLocked(state);
    }
    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(sleep_dt));
    std::this_thread::sleep_until(next_tick);
  }
}

ReferenceTrackingSummary SummarizeReferenceTracking(
    const my_al_ilqr::AutodriveDemoScenario& scenario,
    const Trajectory& trajectory) {
  ReferenceTrackingSummary summary;
  summary.base_cost = scenario.base_problem->TotalCost(trajectory);
  const int horizon = trajectory.Horizon();
  if (horizon < 0) {
    return summary;
  }

  summary.terminal_abs_lateral =
      std::abs(scenario.reference_line->LateralError(trajectory.State(horizon)));
  summary.terminal_abs_heading =
      std::abs(scenario.reference_line->HeadingError(trajectory.State(horizon)));

  const int tail_length = std::max(6, horizon / 4);
  const int start = std::max(0, horizon - tail_length);
  double lateral_sum = 0.0;
  double heading_sum = 0.0;
  int sample_count = 0;
  for (int k = start; k <= horizon; ++k) {
    lateral_sum += std::abs(scenario.reference_line->LateralError(trajectory.State(k)));
    heading_sum += std::abs(scenario.reference_line->HeadingError(trajectory.State(k)));
    ++sample_count;
  }
  if (sample_count > 0) {
    summary.tail_mean_abs_lateral = lateral_sum / static_cast<double>(sample_count);
    summary.tail_mean_abs_heading = heading_sum / static_cast<double>(sample_count);
  }
  return summary;
}

InitialGuessDecision ChooseSimulationInitialGuess(
    const my_al_ilqr::AutodriveDemoScenario& scenario,
    const Vector& ego_state,
    const std::vector<Vector>& warm_start_controls,
    bool use_warm_start) {
  InitialGuessDecision decision;
  if (!use_warm_start) {
    decision.reason = "warm start disabled";
    return decision;
  }
  if (warm_start_controls.empty()) {
    decision.reason = "no previous controls";
    return decision;
  }
  if (static_cast<int>(warm_start_controls.size()) != scenario.base_problem->Horizon()) {
    decision.reason = "warm start length mismatch";
    return decision;
  }

  const Trajectory warm_start_traj =
      scenario.base_problem->Rollout(ego_state, warm_start_controls);
  const Trajectory default_traj =
      scenario.base_problem->Rollout(ego_state, scenario.initial_controls);
  const ReferenceTrackingSummary warm_summary =
      SummarizeReferenceTracking(scenario, warm_start_traj);
  const ReferenceTrackingSummary default_summary =
      SummarizeReferenceTracking(scenario, default_traj);

  const double lateral_recovery_tol =
      std::max(0.25, 0.15 * scenario.vehicle_config.body.width);
  const double heading_recovery_tol = 0.12;
  const bool default_recovered =
      default_summary.terminal_abs_lateral <= lateral_recovery_tol &&
      default_summary.tail_mean_abs_lateral <= 1.25 * lateral_recovery_tol &&
      default_summary.terminal_abs_heading <= heading_recovery_tol;
  const bool warm_lags_reference_return =
      warm_summary.terminal_abs_lateral >
          default_summary.terminal_abs_lateral + lateral_recovery_tol &&
      warm_summary.tail_mean_abs_lateral >
          default_summary.tail_mean_abs_lateral + 0.5 * lateral_recovery_tol;
  const bool warm_heading_residual_large =
      warm_summary.terminal_abs_heading >
      default_summary.terminal_abs_heading + heading_recovery_tol;
  const bool warm_cost_worse =
      warm_summary.base_cost > default_summary.base_cost * 1.1;

  if (default_recovered &&
      (warm_lags_reference_return ||
       (warm_heading_residual_large && warm_cost_worse))) {
    decision.reason = "fallback to default guess: warm start tail deviates from reference";
    return decision;
  }

  decision.use_warm_start = true;
  decision.reason = "warm start accepted";
  return decision;
}

PlanningOutput RunSimulationPlanningStep(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& base_config,
    const std::vector<FrontendObstacle>& obstacles,
    const Vector& ego_state,
    const std::vector<Vector>& warm_start_controls,
    bool use_warm_start,
    int request_serial,
    int step_count_at_request,
    double sim_time,
    double deadline_ms,
    const my_al_ilqr::GlobalGuidanceMemory& previous_guidance_memory) {
  PlanningOutput output;
  output.vehicle_config = vehicle_config;
  output.simulation_step_at_request = step_count_at_request;
  output.request_serial = request_serial;
  output.sim_time_at_request = sim_time;
  output.deadline_ms = deadline_ms;
  try {
    StaticObstacleScenarioConfig config =
        ScenarioConfigForObstacleField(base_config, obstacles, ego_state(0));
    config.initial_x = ego_state(0);
    config.initial_y = ego_state(1);
    config.initial_yaw = ego_state(2);
    config.initial_speed = ego_state(3);
    config.target_longitudinal_position =
        ego_state(0) + config.target_speed * 55 * 0.1;
    output.scenario_config = config;

    const auto t_start = std::chrono::steady_clock::now();
    auto prepared = PreparePlanningScenario(
        vehicle_config, config, obstacles, ego_state(0),
        previous_guidance_memory.valid ? &previous_guidance_memory : nullptr,
        &ego_state);
    auto& scenario = prepared.scenario;

    std::vector<Vector> init_controls;
    const InitialGuessDecision init_decision =
        ChooseSimulationInitialGuess(scenario, ego_state, warm_start_controls, use_warm_start);
    if (init_decision.use_warm_start) {
      init_controls = warm_start_controls;
    } else {
      init_controls = scenario.initial_controls;
    }

    const auto t_solver_start = std::chrono::steady_clock::now();
    const auto solve_result = SolvePreparedScenarioRobustly(
        prepared, ego_state, init_controls);
    const auto t_solver_end = std::chrono::steady_clock::now();
    const auto t_end = t_solver_end;
    const auto& result = *solve_result.result;

    const int horizon = result.trajectory.Horizon();
    output.horizon = horizon;
    output.outer_logs = result.outer_logs;
    output.outer_iterations = static_cast<int>(result.outer_logs.size());
    output.total_inner_iterations = 0;
    for (const auto& log : result.outer_logs) {
      output.total_inner_iterations += log.inner_iterations;
      output.base_cost_history.push_back(log.base_cost);
      output.violation_history.push_back(log.max_violation);
      output.penalty_history.push_back(log.max_penalty);
    }
    output.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    output.deadline_missed =
        output.deadline_ms > 1e-6 && output.planning_time_ms > output.deadline_ms;
    output.guidance_time_ms = prepared.guidance_time_ms;
    output.tracking_time_ms = prepared.tracking_time_ms;
    output.solver_time_ms =
        std::chrono::duration<double, std::milli>(t_solver_end - t_solver_start).count();
    output.initial_cost = scenario.base_problem->TotalCost(
        scenario.base_problem->Rollout(ego_state, init_controls));
    output.final_cost = scenario.base_problem->TotalCost(result.trajectory);
    output.initial_violation =
        scenario.constrained_problem->MaxViolation(
            scenario.base_problem->Rollout(ego_state, init_controls));
    output.final_violation =
        scenario.constrained_problem->MaxViolation(result.trajectory);
    output.converged = result.converged;
    output.best_violation = result.best_violation;
    output.warm_start_reusable =
        result.converged &&
        result.best_violation <= scenario.solver_options.constraint_tolerance;
    output.guidance_used = !prepared.guidance.selected_guidance.empty();
    output.guidance_memory = prepared.guidance.planner_memory;
    output.selected_guidance_topology_id = prepared.guidance.selected_topology_id;
    output.guidance_candidates = std::move(prepared.guidance.candidate_bundle);

    output.x.resize(horizon + 1);
    output.y.resize(horizon + 1);
    output.initial_x.resize(horizon + 1);
    output.initial_y.resize(horizon + 1);
    output.pp_seed_x = prepared.pp_seed_x;
    output.pp_seed_y = prepared.pp_seed_y;
    output.guidance_x.resize(prepared.guidance.selected_guidance.size());
    output.guidance_y.resize(prepared.guidance.selected_guidance.size());
    output.yaw.resize(horizon + 1);
    output.speed.resize(horizon + 1);
    output.accel.resize(horizon);
    output.kappa.resize(horizon);

    const auto initial_traj =
        scenario.base_problem->Rollout(ego_state, init_controls);
    for (int k = 0; k <= horizon; ++k) {
      output.initial_x[k] = initial_traj.State(k)(0);
      output.initial_y[k] = initial_traj.State(k)(1);
      output.x[k] = result.trajectory.State(k)(0);
      output.y[k] = result.trajectory.State(k)(1);
      output.yaw[k] = result.trajectory.State(k)(2);
      output.speed[k] = result.trajectory.State(k)(3);
      if (k < horizon) {
        output.accel[k] = result.trajectory.Control(k)(0);
        output.kappa[k] = std::tan(result.trajectory.Control(k)(1)) /
                          vehicle_config.model.wheelbase;
      }
    }
    for (size_t k = 0; k < prepared.guidance.selected_guidance.size(); ++k) {
      output.guidance_x[k] = prepared.guidance.selected_guidance[k].x;
      output.guidance_y[k] = prepared.guidance.selected_guidance[k].y;
    }

    std::ostringstream ts;
    ts << result.trajectory.State(horizon).transpose();
    output.terminal_state = ts.str();
    output.success = true;
    output.lattice_candidates = std::move(scenario.lattice_candidates);
    output.dp_lattice_nodes = std::move(scenario.dp_lattice_nodes);
    std::ostringstream ss;
    ss << "Sim step "
       << " | total " << output.planning_time_ms << " ms"
       << " / ddl " << output.deadline_ms << " ms"
       << " | guide " << output.guidance_time_ms << " ms"
       << " | track " << output.tracking_time_ms << " ms"
       << " | solver " << output.solver_time_ms << " ms"
       << " | " << (output.deadline_missed ? "deadline-miss" : "on-time")
       << " | " << (result.converged ? "converged" : "NOT converged")
       << " | viol=" << result.best_violation
       << " | init="
       << (init_decision.use_warm_start ? "warm-start"
                                        : prepared.selected_init_controls_label)
       << " | topo=" << output.selected_guidance_topology_id
       << " | " << init_decision.reason;
    output.status = ss.str();
  } catch (const std::exception& e) {
    output.status = e.what();
  }
  return output;
}

void ResetSimulation(FrontendState* state) {
  if (!state) return;
  state->sim.step_count = 0;
  state->sim.steps_since_replan = 0;
  state->sim.manual_steps_remaining = 0;
  state->sim.sim_time = 0.0;
  state->sim.control_phase_time = 0.0;
  state->sim.running = false;
  state->sim.step_pending = false;
  state->sim.animating = false;
  state->sim.anim_idx = 0;
  state->sim.anim_total = 0;
  state->sim.anim_last_time = 0.0;
  state->sim.anim_x.clear();
  state->sim.anim_y.clear();
  state->sim.anim_yaw.clear();
  state->sim.anim_speed.clear();
  state->sim.anim_accel.clear();
  state->sim.anim_steer.clear();
  state->sim.executing_controls.clear();
  state->sim.applied_control = Vector::Zero(2);
  state->sim.applied_control_valid = false;
  state->sim.last_applied_serial = -1;
  state->sim.accepted_plan_count = 0;
  state->sim.dropped_stale_plan_count = 0;
  state->sim.dropped_deadline_plan_count = 0;
  state->sim.last_planning_time_ms = 0.0;
  state->sim.last_guidance_time_ms = 0.0;
  state->sim.last_tracking_time_ms = 0.0;
  state->sim.last_solver_time_ms = 0.0;
  state->sim.last_planning_deadline_ms = 0.0;
  ClearSimulationWarmStart(&state->sim);
  ClearSimulationGuidanceMemory(&state->sim);
  state->sim.history_x.clear();
  state->sim.history_y.clear();
  state->sim.applied_control_time_history.clear();
  state->sim.applied_accel_history.clear();
  state->sim.applied_steer_history.clear();
  state->sim.ego_state = Vector(4);
  state->sim.ego_state << state->scenario_config.initial_x,
      state->scenario_config.initial_y, state->scenario_config.initial_yaw,
      state->scenario_config.initial_speed;
  RebuildSimulationRollout(&state->sim, state->vehicle_config);
  {
    std::lock_guard<std::mutex> lock(state->simulation_mutex);
    const int invalidation_serial = state->sim_runtime.next_request_serial;
    state->sim.next_request_serial = invalidation_serial;
    state->sim.last_requested_serial = invalidation_serial;
    state->planning_request_in_flight = false;
    state->planning_request = SimulationPlanningRequest{};
    state->planning_request_ready = false;
    state->sim_runtime = state->sim;
    state->vehicle_config_runtime = state->vehicle_config;
    state->scenario_config_runtime = state->scenario_config;
    state->obstacles_runtime = state->obstacles;
  }
  state->has_result = false;
}

void SetDefaultState(FrontendState* state) {
  if (!state) {
    return;
  }
  const auto default_config = my_al_ilqr::DefaultVehicleBicycleConfigPath().string();
  std::snprintf(state->config_path.data(), state->config_path.size(), "%s", default_config.c_str());
  state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(default_config);
  InitializeObstacleField(&state->obstacles, state->scenario_config);
  ResetSimulation(state);
}

void LoadConfigFromPath(FrontendState* state) {
  if (!state) {
    return;
  }
  try {
    state->vehicle_config = my_al_ilqr::LoadVehicleBicycleConfig(state->config_path.data());
    std::ostringstream status;
    status << "Loaded config from " << state->config_path.data();
    state->status = status.str();
  } catch (const std::exception& e) {
    state->status = e.what();
  }
}

bool DrawVehicleConfigPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;
  ImGui::SeparatorText("Vehicle Model");
  changed |= SliderInputDouble("Wheelbase", &state->vehicle_config.model.wheelbase, 1.5, 6.0, "%.2f");
  changed |= SliderInputDouble("Min Accel", &state->vehicle_config.model.min_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Max Accel", &state->vehicle_config.model.max_acceleration, -5.0, 5.0, "%.2f");
  changed |= SliderInputDouble("Min Steer", &state->vehicle_config.model.min_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Max Steer", &state->vehicle_config.model.max_steering, -1.2, 1.2, "%.2f");
  changed |= SliderInputDouble("Min Speed", &state->vehicle_config.model.min_speed, -2.0, 20.0, "%.2f");
  changed |= SliderInputDouble("Max Speed", &state->vehicle_config.model.max_speed, -2.0, 20.0, "%.2f");

  ImGui::SeparatorText("Vehicle Body");
  changed |= SliderInputDouble("Length", &state->vehicle_config.body.length, 2.0, 8.0, "%.2f");
  changed |= SliderInputDouble("Width", &state->vehicle_config.body.width, 1.0, 3.0, "%.2f");
  changed |= SliderInputDouble("Rear Axle To Rear", &state->vehicle_config.body.rear_axle_to_rear, 0.2, 3.0, "%.2f");
  changed |= ImGui::SliderInt("Collision Circles", &state->vehicle_config.body.collision_circle_count, 2, 5);
  return changed;
}

bool DrawInitialGuessSetupPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;
  ImGui::SeparatorText("Initial Guess");
  {
    const char* mode_names[] = {"None", "Global Guidance", "SL-DP"};
    int mode_idx = static_cast<int>(state->scenario_config.init_guess_mode);
    if (ImGui::Combo("Init Guess", &mode_idx, mode_names, 3)) {
      state->scenario_config.init_guess_mode =
          static_cast<my_al_ilqr::InitGuessMode>(mode_idx);
      changed = true;
    }
  }

  const bool has_guidance_source =
      state->scenario_config.init_guess_mode != my_al_ilqr::InitGuessMode::kNone;
  if (!has_guidance_source) {
    ImGui::BeginDisabled();
  }
  {
    const char* method_names[] = {
        "Initial Guess Built-In",
        "PP",
        "PP+iLQR",
    };
    int method_idx = static_cast<int>(state->scenario_config.init_controls_mode);
    if (ImGui::Combo("Init Controls", &method_idx, method_names, 3)) {
      state->scenario_config.init_controls_mode =
          static_cast<InitControlsMode>(method_idx);
      changed = true;
    }
  }
  if (!has_guidance_source) {
    ImGui::EndDisabled();
    ImGui::TextWrapped(
        "When Initial Guess is None, the solver uses the scenario default init_controls.");
  }
  return changed;
}

bool DrawObstacleShapePanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;

  ImGui::SeparatorText("Obstacle Shape");
  {
    const char* type_names[] = {"Circle", "Polygon (LSE Smooth)", "Polygon (Exact)"};
    int type_idx = static_cast<int>(state->scenario_config.obstacle_type);
    if (ImGui::Combo("Type", &type_idx, type_names, 3)) {
      state->scenario_config.obstacle_type = static_cast<ObstacleType>(type_idx);
      changed = true;
    }
  }
  if (state->scenario_config.obstacle_type == ObstacleType::kCircle) {
    changed |= SliderInputDouble("Obstacle Radius", &state->scenario_config.obstacle_radius, 0.1, 5.0, "%.2f");
  } else {
    changed |= SliderInputDouble("Half Length", &state->scenario_config.obstacle_half_length, 0.2, 6.0, "%.2f");
    changed |= SliderInputDouble("Half Width", &state->scenario_config.obstacle_half_width, 0.1, 3.0, "%.2f");
    changed |= SliderInputDouble("Obstacle Yaw", &state->scenario_config.obstacle_yaw, -3.2, 3.2, "%.2f");
    if (state->scenario_config.obstacle_type == ObstacleType::kPolygonLSE) {
      changed |= SliderInputDouble("LSE Alpha", &state->scenario_config.lse_alpha, 1.0, 100.0, "%.1f");
    }
  }
  changed |= SliderInputDouble("Road Half Width", &state->scenario_config.road_half_width, 0.6, 6.0, "%.2f");
  ClampObstacleFieldToRoad(state->scenario_config, &state->obstacles);

  return changed;
}

bool DrawPlanningConfigPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;

  ImGui::SeparatorText("Planning Config");
  changed |= SliderInputDouble("Initial Speed", &state->scenario_config.initial_speed, 0.0, 10.0, "%.1f");
  changed |= SliderInputDouble("Target Speed", &state->scenario_config.target_speed, 0.5, 15.0, "%.1f");
  changed |= SliderInputDouble("Max Speed", &state->vehicle_config.model.max_speed, 2.0, 20.0, "%.1f");
  {
    int h = state->scenario_config.planning_horizon;
    if (ImGui::SliderInt("Horizon (steps)", &h, 20, 120)) {
      state->scenario_config.planning_horizon = h;
      changed = true;
    }
  }
  {
    int layers = state->scenario_config.guidance_search_layers;
    if (ImGui::SliderInt("Guidance Layers", &layers, 1, 8)) {
      state->scenario_config.guidance_search_layers = layers;
      changed = true;
    }
  }
  changed |= SliderInputDouble("Jerk Penalty", &state->scenario_config.jerk_weight, 0.0, 10.0, "%.1f");
  changed |= SliderInputDouble("Steer Rate Penalty", &state->scenario_config.steering_rate_weight, 0.0, 10.0, "%.1f");

  {
    const char* deriv_mode_names[] = {"Finite Difference", "Analytical"};
    int deriv_idx = static_cast<int>(state->scenario_config.derivative_mode);
    if (ImGui::Combo("Derivative Mode", &deriv_idx, deriv_mode_names, 2)) {
      state->scenario_config.derivative_mode =
          static_cast<my_al_ilqr::DerivativeMode>(deriv_idx);
      changed = true;
    }
  }

  return changed;
}

bool DrawSimulationPanel(FrontendState* state) {
  if (!state) return false;
  bool changed = false;
  const bool busy = state->planning && state->mode == FrontendMode::kSinglePlan;
  ImGui::SeparatorText("Simulation");
  if (busy) ImGui::BeginDisabled();
  {
    const char* mode_names[] = {"Single Plan", "Simulation (MPC)"};
    int mode_idx = static_cast<int>(state->mode);
    if (ImGui::Combo("Mode", &mode_idx, mode_names, 2)) {
      const FrontendMode previous_mode = state->mode;
      state->mode = static_cast<FrontendMode>(mode_idx);
      if (state->mode == FrontendMode::kSimulation) {
        ResetSimulation(state);
      } else if (previous_mode == FrontendMode::kSimulation) {
        state->sim.running = false;
        state->sim.manual_steps_remaining = 0;
        state->sim.step_pending = false;
        SyncSimulationRuntimeFromSnapshot(state);
      }
      changed = true;
    }
  }
  {
    const char* preset_names[] = {
        "Straight Road", "Staggered Obstacles",
        "Overtaking", "Intersection", "Car Following"};
    int preset_idx = static_cast<int>(state->active_preset);
    if (ImGui::Combo("Scenario", &preset_idx, preset_names, 5)) {
      if (preset_idx == 0) {
        state->active_preset =
            static_cast<my_al_ilqr::ScenarioPreset>(preset_idx);
        ApplyScenarioPreset(state->active_preset,
                            &state->scenario_config, &state->obstacles);
        ResetSimulation(state);
        changed = true;
      } else {
        state->show_coming_soon_popup = true;
      }
    }
  }
  if (state->mode == FrontendMode::kSimulation) {
    ImGui::Spacing();
    if (ImGui::Button("Step")) {
      state->sim.running = false;
      state->sim.manual_steps_remaining = 1;
      state->sim.step_pending = true;
      SyncSimulationRuntimeFromSnapshot(state);
    }
    ImGui::SameLine();
    if (state->sim.running) {
      if (ImGui::Button("Pause")) {
        state->sim.running = false;
        SyncSimulationRuntimeFromSnapshot(state);
      }
    } else {
      if (ImGui::Button("Run")) {
        state->sim.running = true;
        state->sim.step_pending = true;
        SyncSimulationRuntimeFromSnapshot(state);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      ResetSimulation(state);
      state->preview_dirty = true;
      changed = true;
    }
    ImGui::Spacing();
  }
  changed |= DrawPlanningConfigPanel(state);
  changed |= DrawInitialGuessSetupPanel(state);
  changed |= DrawObstacleShapePanel(state);
  if (state->mode == FrontendMode::kSimulation) {
    ImGui::SeparatorText("Simulation Status");
    changed |= ImGui::Checkbox("Warm Start", &state->sim.use_warm_start);
    changed |= ImGui::SliderInt("Replan Interval", &state->sim.replan_steps, 1, 20);
    changed |= SliderInputDouble("Physics dt [s]", &state->sim.physics_tick_dt, 0.005, 0.05, "%.3f");
    ImGui::Text("Control Tick: %.1f Hz  dt=%.2fs", kTrajectoryStepHz, kTrajectoryStepDt);
    ImGui::Text("Physics Tick: %.1f Hz  dt=%.3fs",
                1.0 / std::max(1e-3, state->sim.physics_tick_dt), state->sim.physics_tick_dt);
    ImGui::Text("Step: %d  Time: %.2fs", state->sim.step_count, state->sim.sim_time);
    ImGui::Text("Ego: (%.2f, %.2f) yaw=%.2f v=%.2f",
                state->sim.ego_state(0), state->sim.ego_state(1),
                state->sim.ego_state(2), state->sim.ego_state(3));
    ImGui::Text("Applied u: a=%.2f steer=%.2f",
                state->sim.applied_control.size() == 2 ? state->sim.applied_control(0) : 0.0,
                state->sim.applied_control.size() == 2 ? state->sim.applied_control(1) : 0.0);
    ImGui::Text("Plans: accepted=%d stale_drop=%d deadline_drop=%d",
                state->sim.accepted_plan_count,
                state->sim.dropped_stale_plan_count,
                state->sim.dropped_deadline_plan_count);
    ImGui::Text("Last plan: %.1f ms (G:%.1f T:%.1f S:%.1f) / %.1f",
                state->sim.last_planning_time_ms,
                state->sim.last_guidance_time_ms,
                state->sim.last_tracking_time_ms,
                state->sim.last_solver_time_ms,
                state->sim.last_planning_deadline_ms);
  }
  if (busy) ImGui::EndDisabled();
  return changed;
}

bool DrawScenarioPanel(FrontendState* state) {
  if (!state) {
    return false;
  }
  bool changed = false;

  ImGui::SeparatorText("Obstacle Field");
  if (ImGui::Button("Reseed Field")) {
    InitializeObstacleField(&state->obstacles, state->scenario_config);
    changed = true;
  }
  ImGui::SameLine();
  ImGui::TextUnformatted("Sparse random spacing, left/center/right placement");
  for (int i = 0; i < static_cast<int>(state->obstacles.size()); ++i) {
    auto& obstacle = state->obstacles[i];
    ImGui::PushID(i);
    const std::string header = "Obstacle " + std::to_string(i + 1);
    if (ImGui::TreeNode(header.c_str())) {
      changed |= ImGui::Checkbox("Enabled", &obstacle.enabled);
      changed |= SliderInputDouble("X", &obstacle.x, 0.0, 220.0, "%.2f");
      changed |= SliderInputDouble("Y", &obstacle.y, -6.0, 6.0, "%.2f");
      changed |= SliderInputDouble("Speed X", &obstacle.speed_x, -5.0, 5.0, "%.2f");
      changed |= SliderInputDouble("Speed Y", &obstacle.speed_y, -5.0, 5.0, "%.2f");
      changed |= SliderInputDouble("Yaw", &obstacle.yaw, -3.2, 3.2, "%.2f");
      ImGui::TreePop();
    }
    ImGui::PopID();
  }
  const int enabled_count = static_cast<int>(std::count_if(
      state->obstacles.begin(), state->obstacles.end(),
      [](const FrontendObstacle& obstacle) { return obstacle.enabled; }));
  if (enabled_count == 0 && !state->obstacles.empty()) {
    state->obstacles.front().enabled = true;
    changed = true;
  }

  ImGui::SeparatorText("Initial State");
  changed |= SliderInputDouble("Initial X", &state->scenario_config.initial_x, -5.0, 10.0, "%.2f");
  changed |= SliderInputDouble("Initial Y", &state->scenario_config.initial_y, -4.0, 4.0, "%.2f");
  changed |= SliderInputDouble("Initial Yaw", &state->scenario_config.initial_yaw, -3.2, 3.2, "%.2f");
  changed |= SliderInputDouble("Initial Speed", &state->scenario_config.initial_speed, 0.0, 10.0, "%.2f");

  const auto guess_mode = state->scenario_config.init_guess_mode;
  ImGui::SeparatorText("Planner Settings");
  if (guess_mode == my_al_ilqr::InitGuessMode::kGlobalGuidance) {
    ImGui::TextWrapped("Topology-aware multi-candidate bundle with follow/bypass strategies.");
    changed |= ImGui::SliderInt("Guide Layers", &state->scenario_config.guidance_search_layers, 1, 5);
    changed |= ImGui::SliderInt("Guide Lat Samples",
                                &state->scenario_config.guidance_search_lateral_samples,
                                5, 13);
    changed |= SliderInputDouble("Guide Lat Span", &state->scenario_config.guidance_search_lateral_span,
                                 0.5, 4.0, "%.2f");
    changed |= SliderInputDouble("Guide Max Ratio", &state->scenario_config.guidance_search_max_speed_ratio,
                                 1.0, 1.8, "%.2f");
    changed |= SliderInputDouble("Guide Consistency", &state->scenario_config.guidance_search_consistency_penalty,
                                 0.0, 30.0, "%.2f");
    changed |= SliderInputDouble("Guide Geo Consist",
                                 &state->scenario_config.guidance_search_consistency_geometry_weight,
                                 0.0, 2.0, "%.2f");
    changed |= SliderInputDouble("Guide Path Len", &state->scenario_config.guidance_search_path_length_weight,
                                 0.0, 2.0, "%.2f");
    changed |= SliderInputDouble("Guide Collision", &state->scenario_config.guidance_search_collision_penalty,
                                 100.0, 50000.0, "%.0f");
    changed |= SliderInputDouble("Guide Prox Buffer",
                                 &state->scenario_config.guidance_search_proximity_buffer,
                                 0.0, 4.0, "%.2f");
    changed |= SliderInputDouble("Guide Prox Weight",
                                 &state->scenario_config.guidance_search_proximity_weight,
                                 0.0, 80.0, "%.2f");
    changed |= SliderInputDouble("Guide Follow Band",
                                 &state->scenario_config.guidance_search_follow_lateral_band,
                                 0.2, 2.5, "%.2f");
    changed |= SliderInputDouble("Guide Follow Gap",
                                 &state->scenario_config.guidance_search_follow_base_gap,
                                 0.0, 20.0, "%.2f");
    changed |= SliderInputDouble("Guide Follow Headway",
                                 &state->scenario_config.guidance_search_follow_headway,
                                 0.0, 3.0, "%.2f");
    changed |= SliderInputDouble("Guide Follow Penalty",
                                 &state->scenario_config.guidance_search_follow_penalty,
                                 0.0, 80.0, "%.2f");
    changed |= SliderInputDouble("Guide Bypass Gap",
                                 &state->scenario_config.guidance_search_bypass_lateral_clearance,
                                 0.4, 3.5, "%.2f");
    changed |= SliderInputDouble("Guide Bypass Early Dist",
                                 &state->scenario_config.guidance_search_bypass_anticipation_distance,
                                 2.0, 30.0, "%.2f");
    changed |= SliderInputDouble("Guide Bypass Early W",
                                 &state->scenario_config.guidance_search_bypass_anticipation_weight,
                                 0.0, 80.0, "%.2f");
    changed |= SliderInputDouble("Guide Hold Dist",
                                 &state->scenario_config.guidance_search_post_bypass_hold_distance,
                                 0.0, 25.0, "%.2f");
    changed |= SliderInputDouble("Guide Hold W",
                                 &state->scenario_config.guidance_search_post_bypass_hold_weight,
                                 0.0, 80.0, "%.2f");
    if (ImGui::TreeNode("Guidance Search Weights")) {
      changed |= SliderInputDouble("Guide Lat W", &state->scenario_config.guidance_search_lateral_weight,
                                   0.0, 10.0, "%.2f");
      changed |= SliderInputDouble("Guide Speed W", &state->scenario_config.guidance_search_speed_weight,
                                   0.0, 10.0, "%.2f");
      changed |= SliderInputDouble("Guide Accel W", &state->scenario_config.guidance_search_accel_weight,
                                   0.0, 5.0, "%.2f");
      changed |= SliderInputDouble("Guide Steer W", &state->scenario_config.guidance_search_steering_weight,
                                   0.0, 5.0, "%.2f");
      ImGui::TreePop();
    }
  } else if (guess_mode == my_al_ilqr::InitGuessMode::kSLDP) {
    ImGui::TextWrapped("Frenet SL sampling + DP search. Simpler and fewer parameters.");
    changed |= ImGui::SliderInt("DP Layers", &state->scenario_config.dp_layers, 1, 8);
    changed |= ImGui::SliderInt("DP Lat Samples", &state->scenario_config.dp_lateral_samples, 3, 15);
    changed |= SliderInputDouble("DP Lat Span", &state->scenario_config.dp_lateral_span,
                                 0.5, 4.0, "%.2f");
    changed |= ImGui::SliderInt("DP Speed Samples", &state->scenario_config.dp_speed_samples, 1, 7);
    changed |= SliderInputDouble("DP Max Speed Ratio", &state->scenario_config.dp_max_speed_ratio,
                                 0.5, 1.8, "%.2f");
    changed |= ImGui::SliderInt("DP Max Lat Jump", &state->scenario_config.dp_max_lateral_jump, 1, 5);
    if (ImGui::TreeNode("DP Cost Weights")) {
      changed |= SliderInputDouble("DP Lat W", &state->scenario_config.dp_lateral_weight,
                                   0.0, 10.0, "%.2f");
      changed |= SliderInputDouble("DP Lat Change W", &state->scenario_config.dp_lateral_change_weight,
                                   0.0, 20.0, "%.2f");
      ImGui::SetItemTooltip("Penalizes lateral velocity (dd/dt)^2 — suppresses zigzag");
      changed |= SliderInputDouble("DP Heading W", &state->scenario_config.dp_heading_weight,
                                   0.0, 10.0, "%.2f");
      ImGui::SetItemTooltip("Penalizes heading deviation from reference line");
      changed |= SliderInputDouble("DP Speed W", &state->scenario_config.dp_speed_weight,
                                   0.0, 10.0, "%.2f");
      changed |= SliderInputDouble("DP Accel W", &state->scenario_config.dp_accel_weight,
                                   0.0, 5.0, "%.2f");
      changed |= SliderInputDouble("DP Steer W", &state->scenario_config.dp_steering_weight,
                                   0.0, 5.0, "%.2f");
      ImGui::SetItemTooltip("Penalizes steering rate (jerk) — suppresses oscillation");
      changed |= SliderInputDouble("DP Path Len W", &state->scenario_config.dp_path_length_weight,
                                   0.0, 2.0, "%.2f");
      changed |= SliderInputDouble("DP Collision", &state->scenario_config.dp_collision_penalty,
                                   100.0, 50000.0, "%.0f");
      changed |= SliderInputDouble("DP Prox Buffer", &state->scenario_config.dp_proximity_buffer,
                                   0.0, 6.0, "%.2f");
      changed |= SliderInputDouble("DP Prox Weight", &state->scenario_config.dp_proximity_weight,
                                   0.0, 80.0, "%.2f");
      ImGui::Separator();
      ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Anticipation (early avoidance)");
      changed |= SliderInputDouble("Anticip Dist", &state->scenario_config.dp_anticipation_distance,
                                   2.0, 30.0, "%.1f");
      ImGui::SetItemTooltip("Lookahead distance (m) for proactive avoidance");
      changed |= SliderInputDouble("Anticip Lat Clr", &state->scenario_config.dp_anticipation_lateral_clearance,
                                   0.5, 4.0, "%.2f");
      ImGui::SetItemTooltip("Desired lateral clearance from obstacle center");
      changed |= SliderInputDouble("Anticip Weight", &state->scenario_config.dp_anticipation_weight,
                                   0.0, 40.0, "%.1f");
      ImGui::TreePop();
    }
  } else {
    ImGui::TextWrapped("Using the scenario default init_controls.");
  }
  return changed;
}

}  // namespace

int main() {
  FrontendState state;
  SetDefaultState(&state);
  RefreshPreviewIfNeeded(&state);

  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW.");
  }

  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow* window = glfwCreateWindow(1600, 960, "MY-AL-iLQR ImGui Frontend", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window.");
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  {
    auto try_load = [](const char* rel) -> GLuint {
      auto p = std::filesystem::current_path() / rel;
      GLuint t = LoadTextureFromFile(p.string().c_str());
      return t;
    };
    state.tex_steering_wheel = try_load("assets/steering_wheel.png");
    state.tex_car_top = try_load("assets/car_top_view.png");
  }

  std::future<PlanningOutput> planning_future;
  state.physics_shutdown.store(false);
  state.physics_thread = std::thread(RunPhysicsThread, &state);

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (state.mode == FrontendMode::kSimulation) {
      SyncSimulationSnapshotFromRuntime(&state);
    }

    if (state.planning &&
        planning_future.valid() &&
        planning_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      PlanningOutput output = planning_future.get();
      state.planning = false;
      state.last_best_violation = output.best_violation;
      state.terminal_state = output.terminal_state;
      state.last_csv_path = output.trajectory_csv_path;
      state.has_result = output.success;
      if (output.success) {
        if (state.mode == FrontendMode::kSimulation) {
          std::string sim_status = output.status;
          {
            std::lock_guard<std::mutex> lock(state.simulation_mutex);
            state.planning_request_in_flight = false;
            SimulationState& sim = state.sim_runtime;
            const VehicleBicycleConfig runtime_vehicle = state.vehicle_config_runtime;
            sim.last_planning_time_ms = output.planning_time_ms;
            sim.last_guidance_time_ms = output.guidance_time_ms;
            sim.last_tracking_time_ms = output.tracking_time_ms;
            sim.last_solver_time_ms = output.solver_time_ms;
            sim.last_planning_deadline_ms = output.deadline_ms;
            const double elapsed_time =
                std::max(0.0, sim.sim_time - output.sim_time_at_request);
            const bool superseded =
                output.request_serial >= 0 &&
                output.request_serial < sim.last_requested_serial;
            output.stale_on_arrival =
                superseded ||
                elapsed_time >
                    (static_cast<double>(sim.stale_plan_tolerance_steps) + 0.5) *
                        kTrajectoryStepDt;

            if (output.deadline_missed) {
              sim.dropped_deadline_plan_count += 1;
              sim.step_pending = true;
              sim_status += " | dropped: deadline miss";
            } else if (output.stale_on_arrival) {
              sim.dropped_stale_plan_count += 1;
              sim.step_pending = true;
              sim_status += " | dropped: stale";
            } else {
              if (output.warm_start_reusable) {
                sim.previous_controls.resize(output.horizon);
                for (int k = 0; k < output.horizon; ++k) {
                  Vector ctrl(2);
                  ctrl << output.accel[k],
                      std::atan(output.kappa[k] * runtime_vehicle.model.wheelbase);
                  sim.previous_controls[k] = ctrl;
                }
                sim.warm_start_valid = true;
              } else {
                ClearSimulationWarmStart(&sim);
              }
              if (output.guidance_memory.valid) {
                sim.guidance_memory = output.guidance_memory;
              } else {
                ClearSimulationGuidanceMemory(&sim);
              }
              InstallSimulationAnimation(&sim, runtime_vehicle, output);
              sim_status += " | applied";
            }
          }
          SyncSimulationSnapshotFromRuntime(&state);
          state.status = sim_status;
        } else {
          state.status = output.status;
        }
        state.result = output;
        if (state.mode == FrontendMode::kSinglePlan) {
          state.scene_zoom = 1.0;
          state.scene_pan_x = 0.0;
          state.scene_pan_y = 0.0;
        }
      } else if (state.mode == FrontendMode::kSimulation) {
        {
          std::lock_guard<std::mutex> lock(state.simulation_mutex);
          state.planning_request_in_flight = false;
          ClearSimulationWarmStart(&state.sim_runtime);
          ClearSimulationGuidanceMemory(&state.sim_runtime);
          state.sim_runtime.step_pending = true;
        }
        SyncSimulationSnapshotFromRuntime(&state);
        state.status = output.status;
      } else {
        state.status = output.status;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    RefreshPreviewIfNeeded(&state);

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float controls_width = 340.0f;
    const float content_padding = 16.0f;
    const float right_x = viewport->Pos.x + controls_width + content_padding;
    const float right_width = viewport->Size.x - controls_width - 2.0f * content_padding;
    const float top_height = viewport->Size.y * 0.60f;
    const float bottom_height = viewport->Size.y - top_height - 3.0f * content_padding;
    const float bottom_third_width = (right_width - 2.0f * content_padding) / 3.0f;

    ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x + content_padding, viewport->Pos.y + content_padding),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(controls_width - content_padding, viewport->Size.y - 2.0f * content_padding),
                             ImGuiCond_Always);
    ImGui::Begin("Controls");
    ImGui::InputText("Config Path", state.config_path.data(), state.config_path.size());
    if (ImGui::Button("Load Config")) {
      LoadConfigFromPath(&state);
    }
    ImGui::SameLine();
    if (state.mode == FrontendMode::kSinglePlan) {
      const bool disable_plan_button = state.planning;
      if (disable_plan_button) ImGui::BeginDisabled();
      if (ImGui::Button("Plan")) {
        state.status = "Planning...";
        state.planning = true;
        const std::filesystem::path config_path = state.config_path.data();
        const VehicleBicycleConfig vehicle_config = state.vehicle_config;
        const StaticObstacleScenarioConfig scenario_config = state.scenario_config;
        const std::vector<FrontendObstacle> obstacles = state.obstacles;
        planning_future = std::async(std::launch::async,
                                     [config_path, vehicle_config, scenario_config, obstacles]() {
                                       return RunPlanningJob(
                                           config_path, vehicle_config,
                                           scenario_config, obstacles);
                                     });
      }
      if (disable_plan_button) ImGui::EndDisabled();
    }

    const bool simulation_panel_changed = DrawSimulationPanel(&state);

    if (state.mode == FrontendMode::kSimulation && !state.planning) {
      bool launch_planning = false;
      SimulationPlanningRequest request;
      {
        std::lock_guard<std::mutex> lock(state.simulation_mutex);
        if (state.planning_request_ready) {
          request = state.planning_request;
          ClearSimulationPlanningRequest(&state);
          state.planning_request_in_flight = true;
          launch_planning = true;
        }
      }
      if (launch_planning && request.valid) {
        state.planning = true;
        state.status = "Sim planning...";
        planning_future = std::async(
            std::launch::async,
            [request]() {
              return RunSimulationPlanningStep(
                  request.vehicle_config, request.scenario_config,
                  request.obstacles, request.ego_state,
                  request.warm_start_controls, request.use_warm_start,
                  request.request_serial, request.step_count,
                  request.sim_time, request.deadline_ms,
                  request.guidance_memory);
            });
      }
    }

    ImGui::Separator();
    ImGui::BeginChild("runtime_summary", ImVec2(0.0f, 118.0f), true);
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    ImGui::BulletText("Best violation: %.6f", state.last_best_violation);
    if (state.has_result && state.result.guidance_used) {
      ImGui::BulletText("Guidance topology: %d", state.result.selected_guidance_topology_id);
    }
    if (state.mode == FrontendMode::kSinglePlan) {
      ImGui::BulletText("Trajectory CSV: %s",
                        state.last_csv_path.empty() ? "not written" : state.last_csv_path.string().c_str());
    }
    ImGui::BulletText("Terminal state: %s",
                      state.terminal_state.empty() ? "not available" : state.terminal_state.c_str());
    ImGui::EndChild();

    ImGui::SeparatorText("Solver Logs");
    ImGui::BeginChild("solver_panel", ImVec2(0.0f, 286.0f), true);
    DrawSolverLogPanel(state);
    ImGui::EndChild();

    ImGui::SeparatorText("Guidance");
    ImGui::BeginChild("guidance_panel", ImVec2(0.0f, 304.0f), true);
    DrawGuidancePanel(state);
    ImGui::EndChild();

    bool model_or_scenario_changed = false;
    if (state.planning && state.mode == FrontendMode::kSinglePlan) ImGui::BeginDisabled();
    model_or_scenario_changed |= DrawVehicleConfigPanel(&state);
    model_or_scenario_changed |= DrawScenarioPanel(&state);
    if (state.planning && state.mode == FrontendMode::kSinglePlan) ImGui::EndDisabled();

    const bool params_changed =
        simulation_panel_changed || model_or_scenario_changed;
    if (params_changed) {
      state.preview_dirty = true;
      if (state.mode == FrontendMode::kSinglePlan) {
        state.has_result = false;
        state.scene_zoom = 1.0;
        state.scene_pan_x = 0.0;
        state.scene_pan_y = 0.0;
      } else {
        ResetSimulation(&state);
        state.status = "Simulation reset after context change. Warm start cleared.";
        state.last_best_violation = 0.0;
        state.terminal_state.clear();
      }
      RefreshPreviewIfNeeded(&state);
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + content_padding), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(right_width, top_height - content_padding), ImGuiCond_Always);
    ImGui::Begin("Visualization");
    ImGui::BeginGroup();
    if (ImGui::Button("Reset View")) {
      state.scene_zoom = 1.0;
      state.scene_pan_x = 0.0;
      state.scene_pan_y = 0.0;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Mouse wheel: zoom   Right-drag: pan");
    ImGui::EndGroup();
    ImGui::SameLine();
    const float checkbox_group_width = 640.0f;
    const float checkbox_group_x = std::max(
        ImGui::GetCursorPosX() + 20.0f,
        ImGui::GetWindowContentRegionMax().x - checkbox_group_width);
    ImGui::SetCursorPosX(checkbox_group_x);
    ImGui::BeginGroup();
    ImGui::Checkbox("Ref", &state.show_reference_line);
    ImGui::SameLine();
    ImGui::Checkbox("Obstacle", &state.show_obstacle);
    ImGui::SameLine();
    ImGui::Checkbox("Init Controls", &state.show_initial_guess);
    ImGui::SameLine();
    ImGui::Checkbox("PP Seed", &state.show_pp_seed);
    ImGui::SameLine();
    ImGui::Checkbox("Opt", &state.show_optimized_trajectory);
    ImGui::Checkbox("Rect", &state.show_vehicle_outline);
    ImGui::SameLine();
    ImGui::Checkbox("Circles", &state.show_collision_circles);
    ImGui::SameLine();
    ImGui::Checkbox("Initial Guess", &state.show_selected_guidance);
    ImGui::Checkbox("Candidates", &state.show_guidance_candidates);
    ImGui::SameLine();
    ImGui::Checkbox("DP Nodes", &state.show_dp_lattice_nodes);
    ImGui::EndGroup();
    ImGui::Separator();
    if (state.has_result) {
      DrawSceneView(&state);
    } else if (state.preview.valid) {
      DrawSceneView(&state);
    } else {
      ImGui::TextWrapped("No preview available.");
    }

    if (state.mode == FrontendMode::kSimulation) {
      ImDrawList* overlay = ImGui::GetWindowDrawList();
      const ImVec2 win_pos = ImGui::GetWindowPos();
      const ImVec2 win_size = ImGui::GetWindowSize();
      const float hud_base_x = win_pos.x + 200.0f;
      const float base_y = win_pos.y + win_size.y - 130.0f;

      const float steer_val = state.sim.applied_control.size() == 2
          ? static_cast<float>(state.sim.applied_control(1)) : 0.0f;
      const float accel_val = state.sim.applied_control.size() == 2
          ? static_cast<float>(state.sim.applied_control(0)) : 0.0f;
      const float max_steer = static_cast<float>(state.vehicle_config.model.max_steering);
      const float max_accel = static_cast<float>(state.vehicle_config.model.max_acceleration);
      const float min_accel = static_cast<float>(state.vehicle_config.model.min_acceleration);

      {
        const float r = 42.0f;
        const ImVec2 center(hud_base_x + r + 4, base_y + r);
        const float rot_angle = -steer_val / max_steer * static_cast<float>(M_PI) * 0.75f;

        overlay->AddRectFilled(
            ImVec2(center.x - r - 6, center.y - r - 6),
            ImVec2(center.x + r + 6, center.y + r + 22),
            IM_COL32(10, 12, 18, 160), 8.0f);

        if (state.tex_steering_wheel) {
          AddImageRotated(overlay,
                          static_cast<ImTextureID>(state.tex_steering_wheel),
                          center, r, r, rot_angle);
        } else {
          overlay->AddCircle(center, r, IM_COL32(120, 120, 120, 200), 32, 2.0f);
          const float line_angle = -static_cast<float>(M_PI) / 2.0f + rot_angle;
          const ImVec2 tip(center.x + r * 0.85f * std::cos(line_angle),
                           center.y + r * 0.85f * std::sin(line_angle));
          overlay->AddLine(center, tip, IM_COL32(100, 200, 255, 255), 3.0f);
        }

        char steer_txt[32];
        std::snprintf(steer_txt, sizeof(steer_txt), u8"Steer: %.2f rad", steer_val);
        const ImVec2 txt_size = ImGui::CalcTextSize(steer_txt);
        overlay->AddText(ImVec2(center.x - txt_size.x * 0.5f, center.y + r + 4),
                         IM_COL32(220, 220, 220, 240), steer_txt);
      }

      {
        const float bar_w = 24.0f;
        const float bar_h = 84.0f;
        const float bar_x = hud_base_x + 100.0f;
        const float bar_y = base_y;

        overlay->AddRectFilled(
            ImVec2(bar_x - 6, bar_y - 6),
            ImVec2(bar_x + bar_w + 6, bar_y + bar_h + 22),
            IM_COL32(10, 12, 18, 160), 8.0f);

        overlay->AddRect(ImVec2(bar_x, bar_y), ImVec2(bar_x + bar_w, bar_y + bar_h),
                         IM_COL32(140, 140, 140, 220), 3.0f, 0, 1.5f);

        const float zero_y = bar_y + bar_h * (-min_accel) / (max_accel - min_accel);
        overlay->AddLine(ImVec2(bar_x - 2, zero_y), ImVec2(bar_x + bar_w + 2, zero_y),
                         IM_COL32(200, 200, 200, 180), 1.0f);

        const float fill_ratio = std::clamp(
            (accel_val - min_accel) / (max_accel - min_accel), 0.0f, 1.0f);
        const float fill_y = bar_y + bar_h * (1.0f - fill_ratio);
        const ImU32 accel_color = accel_val >= 0
            ? IM_COL32(80, 200, 120, 220) : IM_COL32(255, 100, 50, 220);
        if (accel_val >= 0) {
          overlay->AddRectFilled(ImVec2(bar_x + 2, fill_y),
                                 ImVec2(bar_x + bar_w - 2, zero_y), accel_color, 2.0f);
        } else {
          overlay->AddRectFilled(ImVec2(bar_x + 2, zero_y),
                                 ImVec2(bar_x + bar_w - 2, fill_y), accel_color, 2.0f);
        }

        char accel_txt[32];
        std::snprintf(accel_txt, sizeof(accel_txt), "%.2f m/s²", accel_val);
        const ImVec2 atxt_size = ImGui::CalcTextSize(accel_txt);
        overlay->AddText(ImVec2(bar_x + bar_w * 0.5f - atxt_size.x * 0.5f, bar_y + bar_h + 4),
                         IM_COL32(220, 220, 220, 240), accel_txt);
      }
    }

    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(right_x, viewport->Pos.y + top_height + content_padding), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(right_width, bottom_height), ImGuiCond_Always);
    ImGui::Begin("Debug");
    ImGui::Columns(state.mode == FrontendMode::kSimulation ? 5 : 3,
                   "debug_columns_top", false);
    {
      SignalPlotOptions options;
      options.fixed_min = state.vehicle_config.model.min_speed;
      options.fixed_max = state.vehicle_config.model.max_speed;
      DrawSignalPlot("Plan Speed", state.has_result ? state.result.speed : std::vector<double>{},
                     IM_COL32(74, 201, 120, 255), options);
    }
    ImGui::NextColumn();
    DrawSignalPlot("Plan Curvature", state.has_result ? state.result.kappa : std::vector<double>{},
                   IM_COL32(83, 131, 255, 255));
    ImGui::NextColumn();
    {
      SignalPlotOptions options;
      options.fixed_min = state.vehicle_config.model.min_acceleration;
      options.fixed_max = state.vehicle_config.model.max_acceleration;
      DrawSignalPlot("Plan Accel", state.has_result ? state.result.accel : std::vector<double>{},
                     IM_COL32(255, 177, 66, 255), options);
    }
    if (state.mode == FrontendMode::kSimulation) {
      ImGui::NextColumn();
      {
        SignalPlotOptions options;
        options.x_values = &state.sim.applied_control_time_history;
        options.x_label = "time [s]";
        options.scroll_latest = true;
        options.scroll_window = kAppliedControlPlotWindowSec;
        options.fixed_min = state.vehicle_config.model.min_acceleration;
        options.fixed_max = state.vehicle_config.model.max_acceleration;
        DrawSignalPlot("Applied Accel", state.sim.applied_accel_history,
                       IM_COL32(255, 177, 66, 255), options);
      }
      ImGui::NextColumn();
      {
        SignalPlotOptions options;
        options.x_values = &state.sim.applied_control_time_history;
        options.x_label = "time [s]";
        options.scroll_latest = true;
        options.scroll_window = kAppliedControlPlotWindowSec;
        options.fixed_min = state.vehicle_config.model.min_steering;
        options.fixed_max = state.vehicle_config.model.max_steering;
        DrawSignalPlot("Applied Steer", state.sim.applied_steer_history,
                       IM_COL32(72, 149, 239, 255), options);
      }
    }
    ImGui::Columns(1);
    ImGui::Separator();
    ImGui::Columns(3, "debug_columns_bottom", false);
    DrawSignalPlot("Base Cost",
                   state.has_result ? state.result.base_cost_history : std::vector<double>{},
                   IM_COL32(255, 120, 120, 255));
    ImGui::NextColumn();
    {
      SignalPlotOptions options;
      options.fixed_min = 0.0;
      DrawSignalPlot("Violation",
                     state.has_result ? state.result.violation_history : std::vector<double>{},
                     IM_COL32(255, 210, 90, 255), options);
    }
    ImGui::NextColumn();
    {
      SignalPlotOptions options;
      options.fixed_min = 0.0;
      DrawSignalPlot("Penalty",
                     state.has_result ? state.result.penalty_history : std::vector<double>{},
                     IM_COL32(180, 120, 255, 255), options);
    }
    ImGui::Columns(1);
    ImGui::End();

    if (state.show_coming_soon_popup) {
      ImGui::OpenPopup("Coming Soon");
      state.show_coming_soon_popup = false;
    }
    if (ImGui::BeginPopupModal("Coming Soon", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Text("This scenario is coming soon!");
      ImGui::Spacing();
      if (ImGui::Button("OK", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    ImGui::Render();
    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  state.physics_shutdown.store(true);
  if (state.physics_thread.joinable()) {
    state.physics_thread.join();
  }
  if (planning_future.valid()) {
    planning_future.wait();
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
