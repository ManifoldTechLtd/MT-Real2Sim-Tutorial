#include <filesystem>
#include <iostream>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  const auto config = my_al_ilqr::LoadVehicleBicycleConfig(
      my_al_ilqr::DefaultVehicleBicycleConfigPath());
  const auto scenario = my_al_ilqr::CreateSingleStaticObstacleTestScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  const std::filesystem::path csv_path = "build/phase13_single_static_obstacle_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, result.trajectory);

  std::cout << "Phase 13 single static obstacle demo\n";
  std::cout << "config path = " << my_al_ilqr::DefaultVehicleBicycleConfigPath() << "\n";
  std::cout << "wheelbase = " << config.model.wheelbase << "\n";
  std::cout << "control bounds = [" << config.model.min_acceleration << ", "
            << config.model.max_acceleration << "] x [" << config.model.min_steering << ", "
            << config.model.max_steering << "]\n";
  std::cout << "speed bounds = [" << config.model.min_speed << ", " << config.model.max_speed
            << "]\n";
  std::cout << "vehicle length = " << config.body.length
            << ", width = " << config.body.width
            << ", rear_axle_to_rear = " << config.body.rear_axle_to_rear
            << ", circle_count = " << config.body.collision_circle_count << "\n";
  std::cout << "collision circles:\n";
  for (const auto& circle : scenario.vehicle_geometry.Circles()) {
    std::cout << "  center_x_body=" << circle.center_x_body
              << ", center_y_body=" << circle.center_y_body
              << ", radius=" << circle.radius << "\n";
  }
  std::cout << "initial violation = " << initial_violation << "\n";
  std::cout << "returned violation = " << result.best_violation << "\n";
  std::cout << "terminal state = "
            << result.trajectory.State(scenario.base_problem->Horizon()).transpose()
            << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";

  return 0;
}
