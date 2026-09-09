#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

#include "autodrive/lattice_initial_guess.hpp"
#include "autodrive/straight_reference_line.hpp"

using namespace my_al_ilqr;

void TestBasicGeneration() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  LatticeInitialGuess lattice(ref_line, 2.8);

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  LatticeObstacleInfo obs;
  obs.center_x = 6.0;
  obs.center_y = 0.0;
  obs.radius = 0.8;
  obs.speed_x = 0.0;

  auto result = lattice.Generate(state, 3.0, 3.5, 0.95, obs, 55, 0.1);

  std::cout << "  Controls size: " << result.controls.size() << std::endl;
  std::cout << "  Best cost: " << result.best_cost << std::endl;
  std::cout << "  Num candidates: " << result.num_candidates << std::endl;
  std::cout << "  Collision free: " << result.num_collision_free << std::endl;

  assert(static_cast<int>(result.controls.size()) == 55);
  assert(result.best_cost < 1.0e17);

  bool has_nonzero_steering = false;
  for (const auto& u : result.controls) {
    if (std::abs(u(1)) > 0.001) {
      has_nonzero_steering = true;
      break;
    }
  }
  assert(has_nonzero_steering && "Lattice should produce non-trivial steering for obstacle avoidance");

  std::cout << "  First 5 controls:" << std::endl;
  for (int k = 0; k < 5; ++k) {
    std::cout << "    k=" << k << ": accel=" << result.controls[k](0)
              << " steer=" << result.controls[k](1) << std::endl;
  }
  std::cout << "  PASS" << std::endl;
}

void TestNoObstacle() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  LatticeInitialGuess lattice(ref_line, 2.8);

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  LatticeObstacleInfo obs;
  obs.center_x = 100.0;
  obs.center_y = 100.0;
  obs.radius = 0.5;

  auto result = lattice.Generate(state, 3.0, 3.5, 0.95, obs, 55, 0.1);

  std::cout << "  Best cost: " << result.best_cost << std::endl;

  bool mostly_straight = true;
  for (const auto& u : result.controls) {
    if (std::abs(u(1)) > 0.3) {
      mostly_straight = false;
      break;
    }
  }
  assert(mostly_straight && "Without obstacle, should prefer straight path");
  std::cout << "  PASS" << std::endl;
}

void TestMovingObstacle() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  LatticeInitialGuess lattice(ref_line, 2.8);

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  LatticeObstacleInfo obs;
  obs.center_x = 6.0;
  obs.center_y = 0.0;
  obs.radius = 0.8;
  obs.speed_x = 3.0;

  auto result = lattice.Generate(state, 3.0, 3.5, 0.95, obs, 55, 0.1);

  std::cout << "  Best cost (moving obs): " << result.best_cost << std::endl;
  std::cout << "  Collision free: " << result.num_collision_free << std::endl;

  assert(static_cast<int>(result.controls.size()) == 55);
  assert(result.best_cost < 1.0e17);
  std::cout << "  PASS" << std::endl;
}

void TestMultiLayer() {
  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  LatticeInitialGuess lattice(ref_line, 2.8);

  Vector state(4);
  state << 0.0, 0.0, 0.0, 2.5;

  LatticeObstacleInfo obs;
  obs.center_x = 6.0;
  obs.center_y = 0.0;
  obs.radius = 0.8;

  LatticeConfig config;
  config.num_layers = 3;

  auto result = lattice.Generate(state, 3.0, 3.5, 0.95, obs, 55, 0.1, config);

  std::cout << "  Best cost (3 layers): " << result.best_cost << std::endl;
  assert(static_cast<int>(result.controls.size()) == 55);
  assert(result.best_cost < 1.0e17);
  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Lattice Initial Guess Tests ===" << std::endl;

  std::cout << "Test 1: Basic generation with static obstacle" << std::endl;
  TestBasicGeneration();

  std::cout << "Test 2: No obstacle (far away)" << std::endl;
  TestNoObstacle();

  std::cout << "Test 3: Moving obstacle" << std::endl;
  TestMovingObstacle();

  std::cout << "Test 4: Multi-layer (3 layers)" << std::endl;
  TestMultiLayer();

  std::cout << "\nAll lattice tests passed!" << std::endl;
  return 0;
}
