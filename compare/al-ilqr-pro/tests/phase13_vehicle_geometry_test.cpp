#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"

namespace {

bool NearlyEqual(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}

double MinObstacleMargin(const my_al_ilqr::AutodriveDemoScenario& scenario,
                         const my_al_ilqr::Trajectory& trajectory) {
  double min_margin = 1e9;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    const auto centers = scenario.vehicle_geometry.EvaluateWorldCenters(trajectory.State(k));
    for (int i = 0; i < scenario.vehicle_geometry.CircleCount(); ++i) {
      const double dx = centers[i](0) - scenario.obstacle_centers[k](0);
      const double dy = centers[i](1) - scenario.obstacle_centers[k](1);
      const double safe_distance =
          scenario.obstacle_radius + scenario.vehicle_geometry.Circles()[i].radius;
      const double margin = dx * dx + dy * dy - safe_distance * safe_distance;
      min_margin = std::min(min_margin, margin);
    }
  }
  return min_margin;
}

}  // namespace

int main() {
  const auto config = my_al_ilqr::LoadVehicleBicycleConfig(
      my_al_ilqr::DefaultVehicleBicycleConfigPath());
  assert(NearlyEqual(config.model.wheelbase, 2.8));
  assert(config.body.collision_circle_count == 2);

  my_al_ilqr::VehicleCircleApproximation default_geometry(config.body);
  assert(default_geometry.CircleCount() == 2);
  assert(NearlyEqual(default_geometry.Circles()[0].radius, 0.95));
  assert(NearlyEqual(default_geometry.Circles()[0].center_x_body, -0.05));
  assert(NearlyEqual(default_geometry.Circles()[1].center_x_body, 2.65));

  const std::filesystem::path tmp_path =
      std::filesystem::path("build") / "phase13_vehicle_geometry_override.ini";
  std::ofstream tmp_file(tmp_path);
  tmp_file << "[bicycle_model]\n";
  tmp_file << "wheelbase = 2.9\n";
  tmp_file << "min_acceleration = -2.0\n";
  tmp_file << "max_acceleration = 1.5\n";
  tmp_file << "min_steering = -0.5\n";
  tmp_file << "max_steering = 0.5\n";
  tmp_file << "min_speed = 0.0\n";
  tmp_file << "max_speed = 8.0\n\n";
  tmp_file << "[vehicle_body]\n";
  tmp_file << "length = 4.8\n";
  tmp_file << "width = 2.0\n";
  tmp_file << "rear_axle_to_rear = 1.1\n";
  tmp_file << "collision_circle_count = 4\n";
  tmp_file.close();

  const auto override_config = my_al_ilqr::LoadVehicleBicycleConfig(tmp_path);
  my_al_ilqr::VehicleCircleApproximation override_geometry(override_config.body);
  assert(override_geometry.CircleCount() == 4);
  assert(NearlyEqual(override_geometry.Circles().front().center_x_body, -0.1));
  assert(NearlyEqual(override_geometry.Circles().back().center_x_body, 2.7));

  const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  my_al_ilqr::ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  assert(result.best_violation < initial_violation);
  assert(result.best_violation <= 0.25);
  assert(result.trajectory.State(scenario.base_problem->Horizon())(0) >= 10.0);
  assert(MinObstacleMargin(scenario, result.trajectory) >= -0.18);

  return 0;
}
