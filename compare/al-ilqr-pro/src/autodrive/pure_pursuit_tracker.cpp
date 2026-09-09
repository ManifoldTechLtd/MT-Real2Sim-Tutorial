#include "autodrive/pure_pursuit_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace my_al_ilqr {

namespace {

double Wrap(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

GuidancePoint LerpPath(const std::vector<GuidancePoint>& g, double s,
                        int hint) {
  const int n = static_cast<int>(g.size());
  if (n == 0) return {};
  if (s <= g.front().s) return g.front();
  if (s >= g.back().s) return g.back();

  int lo = std::clamp(hint, 0, n - 2);
  while (lo < n - 2 && g[lo + 1].s < s) ++lo;
  while (lo > 0 && g[lo].s > s) --lo;

  const double ds = g[lo + 1].s - g[lo].s;
  const double t = (ds > 1e-9) ? std::clamp((s - g[lo].s) / ds, 0.0, 1.0)
                                : 0.0;
  GuidancePoint p;
  p.x = (1.0 - t) * g[lo].x + t * g[lo + 1].x;
  p.y = (1.0 - t) * g[lo].y + t * g[lo + 1].y;
  p.yaw = Wrap(g[lo].yaw + t * Wrap(g[lo + 1].yaw - g[lo].yaw));
  p.speed = (1.0 - t) * g[lo].speed + t * g[lo + 1].speed;
  p.s = s;
  p.t = (1.0 - t) * g[lo].t + t * g[lo + 1].t;
  return p;
}

double ProjectOnSegments(const std::vector<GuidancePoint>& g, double px,
                          double py, int lo, int hi) {
  const int n = static_cast<int>(g.size());
  lo = std::max(lo, 0);
  hi = std::min(hi, n - 1);
  double best_s = g[lo].s;
  double best_d2 = 1e18;
  for (int i = lo; i < hi; ++i) {
    double sx = g[i + 1].x - g[i].x;
    double sy = g[i + 1].y - g[i].y;
    double len2 = sx * sx + sy * sy;
    double t = 0.0;
    if (len2 > 1e-12)
      t = std::clamp(((px - g[i].x) * sx + (py - g[i].y) * sy) / len2,
                      0.0, 1.0);
    double qx = g[i].x + t * sx - px;
    double qy = g[i].y + t * sy - py;
    double d2 = qx * qx + qy * qy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = g[i].s + t * (g[i + 1].s - g[i].s);
    }
  }
  return best_s;
}

int SegIdx(const std::vector<GuidancePoint>& g, double s, int hint) {
  const int n = static_cast<int>(g.size());
  int idx = std::clamp(hint, 0, n - 2);
  while (idx < n - 2 && g[idx + 1].s < s) ++idx;
  while (idx > 0 && g[idx].s > s) --idx;
  return idx;
}

}  // namespace

PPQuery PurePursuitTracker::Query(const std::vector<GuidancePoint>& g, int gs,
                                   double px, double py, double heading,
                                   int last_near, double v_est, double Ld_base,
                                   double /*dt*/) {
  const int search_hi = std::min(gs - 1, last_near + 40);
  double s_ego = ProjectOnSegments(g, px, py, last_near, search_hi);
  s_ego = std::max(s_ego, g[last_near].s);

  int nearest = SegIdx(g, s_ego, last_near);

  double remaining = g[gs - 1].s - s_ego;
  double Ld_want = std::max(0.5, std::min(Ld_base, std::max(0.1, remaining)));
  double s_target = std::min(s_ego + Ld_want, g[gs - 1].s);
  GuidancePoint target = LerpPath(g, s_target, nearest);

  int look_idx = SegIdx(g, s_target, nearest);
  double target_kappa = 0.0;
  if (look_idx > 0 && look_idx < gs - 1) {
    double ds_local = g[look_idx + 1].s - g[look_idx - 1].s;
    if (ds_local > 1e-6)
      target_kappa =
          Wrap(g[look_idx + 1].yaw - g[look_idx - 1].yaw) / ds_local;
  }

  double dx = target.x - px, dy = target.y - py;
  double Ld = std::sqrt(dx * dx + dy * dy);
  double alpha = Wrap(std::atan2(dy, dx) - heading);
  return {nearest,     look_idx,    target.x, target.y, target.yaw,
          target_kappa, alpha,       Ld,       target.speed, remaining};
}

PurePursuitResult PurePursuitTracker::Track(
    const std::vector<GuidancePoint>& guidance, const Vector& x0,
    std::shared_ptr<DynamicsModel> dynamics, const Vector& u_lb,
    const Vector& u_ub, int horizon, double dt, double wheelbase) {
  PurePursuitResult result;
  const int gs = static_cast<int>(guidance.size());
  if (gs < 2 || horizon <= 0) return result;

  const int sdim = dynamics->StateDim();
  const int udim = dynamics->ControlDim();
  result.states.resize(horizon + 1);
  result.controls.resize(horizon);
  result.states[0] = x0;

  const bool is_bicycle = (sdim == 4 && udim == 2 && wheelbase > 0.01);
  const bool is_unicycle = (sdim == 3 && udim == 2 && !is_bicycle);

  const GuidancePoint& endpoint = guidance.back();

  Vector st = x0;

  for (int k = 0; k < horizon; ++k) {
    double v_est, heading;
    if (is_bicycle) {
      v_est = st(3);
      heading = st(2);
    } else if (is_unicycle) {
      v_est = 0.0;
      heading = st(2);
    } else {
      v_est = std::sqrt(st(2) * st(2) + st(3) * st(3));
      heading = std::atan2(st(3), st(2));
    }

    int nearest_idx = 0;
    {
      double best_d2 = 1e18;
      for (int i = 0; i < gs; ++i) {
        double ddx = guidance[i].x - st(0);
        double ddy = guidance[i].y - st(1);
        double d2 = ddx * ddx + ddy * ddy;
        if (d2 < best_d2) {
          best_d2 = d2;
          nearest_idx = i;
        }
      }
    }

    double Lfw;
    if (is_bicycle) {
      Lfw = std::max(2.0, std::abs(v_est) * 0.5);
    } else {
      Lfw = std::max(0.5, std::abs(v_est) * 0.4);
    }

    double cos_h = std::cos(heading);
    double sin_h = std::sin(heading);
    double fwd_x = endpoint.x;
    double fwd_y = endpoint.y;
    for (int i = 0; i < gs; ++i) {
      double dx = guidance[i].x - st(0);
      double dy = guidance[i].y - st(1);
      double along = dx * cos_h + dy * sin_h;
      if (along <= 0.0) continue;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < Lfw) continue;
      fwd_x = guidance[i].x;
      fwd_y = guidance[i].y;
      break;
    }

    double dx = fwd_x - st(0);
    double dy = fwd_y - st(1);
    double Ld = std::sqrt(dx * dx + dy * dy);
    double eta = Wrap(std::atan2(dy, dx) - heading);

    double remaining_s = guidance.back().s - guidance[nearest_idx].s;
    double remaining_t = (horizon - k) * dt;
    double desired_speed =
        (remaining_t > 0.01) ? remaining_s / remaining_t : 0.0;

    Vector u = Vector::Zero(udim);
    if (is_bicycle) {
      double steer =
          (Ld > 0.05)
              ? std::atan(2.0 * wheelbase * std::sin(eta) / Ld)
              : 0.0;
      u(0) = 2.5 * (desired_speed - v_est);
      u(1) = steer;
    } else if (is_unicycle) {
      double omega =
          (Ld > 0.05)
              ? 2.0 * desired_speed * std::sin(eta) / Ld
              : 0.0;
      u(0) = desired_speed;
      u(1) = omega;
    } else {
      double dir = std::atan2(dy, dx);
      u(0) = 2.0 * dx + 3.0 * (desired_speed * std::cos(dir) - st(2));
      u(1) = 2.0 * dy + 3.0 * (desired_speed * std::sin(dir) - st(3));
    }

    for (int i = 0; i < udim; ++i)
      u(i) = std::clamp(u(i), u_lb(i), u_ub(i));

    result.controls[k] = u;
    st = dynamics->NextState(st, u, dt);
    result.states[k + 1] = st;

    double err_x = st(0) - guidance[nearest_idx].x;
    double err_y = st(1) - guidance[nearest_idx].y;
    result.max_pos_error =
        std::max(result.max_pos_error,
                 std::sqrt(err_x * err_x + err_y * err_y));
  }

  return result;
}

}  // namespace my_al_ilqr
