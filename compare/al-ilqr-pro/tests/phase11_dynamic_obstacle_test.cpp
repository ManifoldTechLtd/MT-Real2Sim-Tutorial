#include <cassert>
#include <cmath>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  const auto scenario = my_al_ilqr::CreateDynamicObstacleDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  assert(!result.outer_logs.empty());
  assert(result.best_violation < initial_violation);
  assert(result.best_violation <= 0.12);

  double min_clearance_margin = 1e9;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    const auto& obstacle = scenario.obstacle_centers[k];
    const auto centers = scenario.vehicle_geometry.EvaluateWorldCenters(result.trajectory.State(k));
    for (int i = 0; i < scenario.vehicle_geometry.CircleCount(); ++i) {
      const double dx = centers[i](0) - obstacle(0);
      const double dy = centers[i](1) - obstacle(1);
      const double safe_distance =
          scenario.obstacle_radius + scenario.vehicle_geometry.Circles()[i].radius;
      const double margin = dx * dx + dy * dy - safe_distance * safe_distance;
      min_clearance_margin = std::min(min_clearance_margin, margin);
    }
  }

  assert(min_clearance_margin >= -0.12);
  assert(result.trajectory.State(scenario.base_problem->Horizon())(0) >= 10.0);

  return 0;
}
