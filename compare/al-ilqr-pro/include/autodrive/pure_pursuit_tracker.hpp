#pragma once

#include <memory>
#include <vector>

#include "autodrive/global_guidance_planner.hpp"
#include "core/types.hpp"
#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

struct PPQuery {
  int nearest = 0;
  int look_idx = 0;
  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  double target_kappa = 0.0;
  double alpha = 0.0;
  double Ld = 0.0;
  double ref_speed = 0.0;
  double remaining = 0.0;
};

struct PurePursuitResult {
  std::vector<Vector> states;
  std::vector<Vector> controls;
  double max_pos_error = 0.0;
};

class PurePursuitTracker {
 public:
  static PPQuery Query(const std::vector<GuidancePoint>& guidance,
                       int guidance_size, double px, double py, double heading,
                       int last_near, double v_est, double Ld_base, double dt);

  static PurePursuitResult Track(
      const std::vector<GuidancePoint>& guidance,
      const Vector& x0,
      std::shared_ptr<DynamicsModel> dynamics,
      const Vector& u_lb, const Vector& u_ub,
      int horizon, double dt,
      double wheelbase = 0.0);
};

}  // namespace my_al_ilqr
