#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "autodrive/demo_scenario.hpp"
#include "autodrive/global_guidance_planner.hpp"

using namespace my_al_ilqr;

VehicleCircleApproximation DefaultVehicleGeometry() {
  VehicleBodyConfig body;
  body.length = 4.8;
  body.width = 1.9;
  body.rear_axle_to_rear = 1.0;
  body.collision_circle_count = 3;
  return VehicleCircleApproximation(body);
}

void TestGuidanceBundleAndControls() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.0, 0.0, 0.0, 0.0, 55, 0.1));
  obstacles.push_back(MakeConstantVelocityPrediction(1, 0.8, 10.5, 1.4, -0.6, 0.0, 55, 0.1));

  GlobalGuidanceResult result =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1);

  std::cout << "  candidates=" << result.candidate_bundle.size()
            << " topology=" << result.selected_topology_id
            << " best_cost=" << result.best_cost << std::endl;

  assert(!result.used_fallback);
  assert(static_cast<int>(result.controls.size()) == 55);
  assert(static_cast<int>(result.selected_guidance.size()) == 56);
  assert(!result.candidate_bundle.empty());
  assert(result.selected_topology_id > 0);
}

void TestTopologyMemoryReuse() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.0, 0.0, 0.0, 0.0, 55, 0.1));
  obstacles.push_back(MakeConstantVelocityPrediction(1, 0.8, 9.5, -1.2, 0.2, 0.0, 55, 0.1));

  const GlobalGuidanceResult first =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1);
  const GlobalGuidanceResult second =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1, {}, &first.planner_memory);

  std::cout << "  first_topology=" << first.selected_topology_id
            << " second_topology=" << second.selected_topology_id << std::endl;

  assert(first.selected_topology_id > 0);
  assert(second.selected_topology_id == first.selected_topology_id);
}

void TestTopologyMemoryReuseAfterSmallPredictionShift() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> first_obstacles;
  first_obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.0, 0.0, 0.0, 0.0, 55, 0.1));
  first_obstacles.push_back(MakeConstantVelocityPrediction(1, 0.8, 9.5, -1.0, 0.1, 0.0, 55, 0.1));

  std::vector<PredictedObstacle> shifted_obstacles;
  shifted_obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.1, 0.0, 0.0, 0.0, 55, 0.1));
  shifted_obstacles.push_back(MakeConstantVelocityPrediction(1, 0.8, 9.6, -0.95, 0.1, 0.0, 55, 0.1));

  const GlobalGuidanceResult first =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, first_obstacles, 55, 0.1);
  const GlobalGuidanceResult second =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, shifted_obstacles, 55, 0.1, {}, &first.planner_memory);

  std::cout << "  shifted first_topology=" << first.selected_topology_id
            << " second_topology=" << second.selected_topology_id << std::endl;

  assert(first.selected_topology_id > 0);
  assert(second.selected_topology_id == first.selected_topology_id);
}

void TestScenarioBuildWithGuidanceTracking() {
  const VehicleBicycleConfig vehicle_config =
      LoadVehicleBicycleConfig(DefaultVehicleBicycleConfigPath());

  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, vehicle_config.model.wheelbase);

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.0, 0.0, 0.0, 0.0, 55, 0.1));

  const GlobalGuidanceResult guidance =
      planner.Generate(state, 3.0, 3.5, VehicleCircleApproximation(vehicle_config.body),
                       obstacles, 55, 0.1);

  StaticObstacleScenarioConfig config;
  config.use_guidance_tracking = true;
  auto scenario =
      CreateSingleStaticObstacleTestScenario(vehicle_config, config, &guidance.selected_guidance);
  scenario.initial_controls = guidance.controls;

  const Trajectory trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double cost = scenario.base_problem->TotalCost(trajectory);

  std::cout << "  guidance tracking cost=" << cost << std::endl;

  assert(static_cast<int>(scenario.initial_controls.size()) == scenario.base_problem->Horizon());
  assert(std::isfinite(cost));
  assert(!scenario.selected_guidance.empty());
}

void TestCustomGuidanceConfigProducesValidSolution() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 6.0, 0.0, 0.0, 0.0, 55, 0.1));
  obstacles.push_back(MakeConstantVelocityPrediction(1, 0.8, 11.0, 1.5, -0.8, 0.0, 55, 0.1));

  GlobalGuidanceConfig config;
  config.num_layers = 3;
  config.lateral_offsets = {-2.0, -1.0, 0.0, 1.0, 2.0};
  config.speed_ratios = {0.0, 0.6, 1.0, 1.35};
  config.consistency_penalty = 4.0;
  config.consistency_geometry_weight = 0.3;

  const GlobalGuidanceResult result =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1, config);

  std::cout << "  custom config candidates=" << result.candidate_bundle.size()
            << " cost=" << result.best_cost << std::endl;

  assert(!result.used_fallback);
  assert(!result.candidate_bundle.empty());
  assert(static_cast<int>(result.controls.size()) == 55);
}

void TestCloseObstacleDoesNotSelectStraightSafeCandidate() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeConstantVelocityPrediction(0, 0.8, 4.2, 0.0, 0.0, 0.0, 55, 0.1));

  const GlobalGuidanceResult result =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1);

  std::cout << "  close obstacle selected signature="
            << result.candidate_bundle.front().signature.key
            << " safe=" << result.candidate_bundle.front().collision_free << std::endl;

  assert(!result.candidate_bundle.empty());
  assert(!(result.candidate_bundle.front().signature.key == "0:RA" &&
           result.candidate_bundle.front().collision_free));
}

void TestCloseBoxObstacleDoesNotSelectStraightSafeCandidate() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  GlobalGuidancePlanner planner(ref_line, 2.8);
  const VehicleCircleApproximation vehicle_geometry = DefaultVehicleGeometry();

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  std::vector<PredictedObstacle> obstacles;
  obstacles.push_back(MakeBoxConstantVelocityPrediction(
      0, 2.3, 0.9, 0.0, 6.0, 0.0, 0.0, 0.0, 55, 0.1));

  GlobalGuidanceConfig config;
  config.lateral_offsets = {-2.4, -1.2, 0.0, 1.2, 2.4};

  const GlobalGuidanceResult result =
      planner.Generate(state, 3.0, 3.5, vehicle_geometry, obstacles, 55, 0.1, config);

  std::cout << "  close box obstacle selected signature="
            << result.candidate_bundle.front().signature.key
            << " safe=" << result.candidate_bundle.front().collision_free << std::endl;

  assert(!result.candidate_bundle.empty());
  assert(!(result.candidate_bundle.front().signature.key == "0:RA" &&
           result.candidate_bundle.front().collision_free));
}

int main() {
  std::cout << "=== Global Guidance Tests ===" << std::endl;

  std::cout << "Test 1: guidance bundle and controls" << std::endl;
  TestGuidanceBundleAndControls();

  std::cout << "Test 2: topology memory reuse" << std::endl;
  TestTopologyMemoryReuse();

  std::cout << "Test 3: scenario build with guidance tracking" << std::endl;
  TestScenarioBuildWithGuidanceTracking();

  std::cout << "Test 4: topology memory reuse after small prediction shift" << std::endl;
  TestTopologyMemoryReuseAfterSmallPredictionShift();

  std::cout << "Test 5: custom guidance config" << std::endl;
  TestCustomGuidanceConfigProducesValidSolution();

  std::cout << "Test 6: close obstacle should not keep straight-safe candidate" << std::endl;
  TestCloseObstacleDoesNotSelectStraightSafeCandidate();

  std::cout << "Test 7: close box obstacle should not keep straight-safe candidate" << std::endl;
  TestCloseBoxObstacleDoesNotSelectStraightSafeCandidate();

  std::cout << "\nAll global guidance tests passed!" << std::endl;
  return 0;
}
