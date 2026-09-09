#include "autodrive/lane_tracking_cost.hpp"

#include <stdexcept>

namespace my_al_ilqr {

LaneTrackingCost::LaneTrackingCost(std::shared_ptr<StraightReferenceLine> reference_line,
                                   double target_speed,
                                   double target_longitudinal_position,
                                   double stage_lateral_weight,
                                   double stage_heading_weight,
                                   double stage_speed_weight,
                                   double stage_accel_weight,
                                   double stage_steering_weight,
                                   double terminal_longitudinal_weight,
                                   double terminal_lateral_weight,
                                   double terminal_heading_weight,
                                   double terminal_speed_weight)
    : reference_line_(std::move(reference_line)),
      target_speed_(target_speed),
      target_longitudinal_position_(target_longitudinal_position),
      stage_lateral_weight_(stage_lateral_weight),
      stage_heading_weight_(stage_heading_weight),
      stage_speed_weight_(stage_speed_weight),
      stage_accel_weight_(stage_accel_weight),
      stage_steering_weight_(stage_steering_weight),
      terminal_longitudinal_weight_(terminal_longitudinal_weight),
      terminal_lateral_weight_(terminal_lateral_weight),
      terminal_heading_weight_(terminal_heading_weight),
      terminal_speed_weight_(terminal_speed_weight) {
  if (!reference_line_) {
    throw std::invalid_argument("LaneTrackingCost requires a valid reference line.");
  }
}

double LaneTrackingCost::StageCost(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("LaneTrackingCost::StageCost received an input with invalid dimensions.");
  }

  const double lateral_error = reference_line_->LateralError(state);
  const double heading_error = reference_line_->HeadingError(state);
  const double speed_error = state(3) - target_speed_;

  return 0.5 * stage_lateral_weight_ * lateral_error * lateral_error +
         0.5 * stage_heading_weight_ * heading_error * heading_error +
         0.5 * stage_speed_weight_ * speed_error * speed_error +
         0.5 * stage_accel_weight_ * control(0) * control(0) +
         0.5 * stage_steering_weight_ * control(1) * control(1);
}

double LaneTrackingCost::TerminalCost(const Vector& state) const {
  if (state.size() != StateDim()) {
    throw std::invalid_argument(
        "LaneTrackingCost::TerminalCost received a state with invalid dimensions.");
  }

  const double longitudinal_error =
      reference_line_->LongitudinalPosition(state) - target_longitudinal_position_;
  const double lateral_error = reference_line_->LateralError(state);
  const double heading_error = reference_line_->HeadingError(state);
  const double speed_error = state(3) - target_speed_;

  return 0.5 * terminal_longitudinal_weight_ * longitudinal_error * longitudinal_error +
         0.5 * terminal_lateral_weight_ * lateral_error * lateral_error +
         0.5 * terminal_heading_weight_ * heading_error * heading_error +
         0.5 * terminal_speed_weight_ * speed_error * speed_error;
}

CostExpansion LaneTrackingCost::AnalyticalStageCostExpansion(const Vector& state,
                                                             const Vector& control) const {
  const double h = reference_line_->Heading();
  const double sin_h = std::sin(h);
  const double cos_h = std::cos(h);

  const double e_lat = reference_line_->LateralError(state);
  const double e_hdg = reference_line_->HeadingError(state);
  const double e_spd = state(3) - target_speed_;

  // Jacobians of error terms w.r.t. state: de/dx (each is 1x4, stored as 4x1 column)
  Vector de_lat(4);
  de_lat << -sin_h, cos_h, 0.0, 0.0;
  Vector de_hdg(4);
  de_hdg << 0.0, 0.0, 1.0, 0.0;
  Vector de_spd(4);
  de_spd << 0.0, 0.0, 0.0, 1.0;

  Vector lx = stage_lateral_weight_ * e_lat * de_lat +
              stage_heading_weight_ * e_hdg * de_hdg +
              stage_speed_weight_ * e_spd * de_spd;
  Vector lu(2);
  lu << stage_accel_weight_ * control(0), stage_steering_weight_ * control(1);

  Matrix lxx = stage_lateral_weight_ * de_lat * de_lat.transpose() +
               stage_heading_weight_ * de_hdg * de_hdg.transpose() +
               stage_speed_weight_ * de_spd * de_spd.transpose();

  Matrix luu = Matrix::Zero(2, 2);
  luu(0, 0) = stage_accel_weight_;
  luu(1, 1) = stage_steering_weight_;

  Matrix lux = Matrix::Zero(2, 4);

  return CostExpansion{lx, lu, lxx, lux, luu};
}

std::pair<Vector, Matrix> LaneTrackingCost::AnalyticalTerminalCostExpansion(
    const Vector& state) const {
  const double h = reference_line_->Heading();
  const double sin_h = std::sin(h);
  const double cos_h = std::cos(h);

  const double e_lon = reference_line_->LongitudinalPosition(state) - target_longitudinal_position_;
  const double e_lat = reference_line_->LateralError(state);
  const double e_hdg = reference_line_->HeadingError(state);
  const double e_spd = state(3) - target_speed_;

  Vector de_lon(4);
  de_lon << cos_h, sin_h, 0.0, 0.0;
  Vector de_lat(4);
  de_lat << -sin_h, cos_h, 0.0, 0.0;
  Vector de_hdg(4);
  de_hdg << 0.0, 0.0, 1.0, 0.0;
  Vector de_spd(4);
  de_spd << 0.0, 0.0, 0.0, 1.0;

  Vector lx = terminal_longitudinal_weight_ * e_lon * de_lon +
              terminal_lateral_weight_ * e_lat * de_lat +
              terminal_heading_weight_ * e_hdg * de_hdg +
              terminal_speed_weight_ * e_spd * de_spd;

  Matrix lxx = terminal_longitudinal_weight_ * de_lon * de_lon.transpose() +
               terminal_lateral_weight_ * de_lat * de_lat.transpose() +
               terminal_heading_weight_ * de_hdg * de_hdg.transpose() +
               terminal_speed_weight_ * de_spd * de_spd.transpose();

  return {lx, lxx};
}

}  // namespace my_al_ilqr
