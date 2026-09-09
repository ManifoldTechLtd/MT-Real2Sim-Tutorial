#include "autodrive/polygon_collision_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

PolygonCollisionConstraint::PolygonCollisionConstraint(
    double vehicle_length, double vehicle_width, double rear_axle_to_rear,
    ConvexPolygon obstacle_polygon, double safety_margin)
    : vehicle_length_(vehicle_length),
      vehicle_width_(vehicle_width),
      rear_axle_to_rear_(rear_axle_to_rear),
      obstacle_polygon_(std::move(obstacle_polygon)),
      safety_margin_(safety_margin) {
  if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0) {
    throw std::invalid_argument(
        "PolygonCollisionConstraint requires positive vehicle dimensions.");
  }
}

Vector PolygonCollisionConstraint::Evaluate(const Vector& state,
                                            const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "PolygonCollisionConstraint: invalid input dimensions.");
  }

  ConvexPolygon vehicle = MakeVehiclePolygon(
      state(0), state(1), state(2), vehicle_length_, vehicle_width_,
      rear_axle_to_rear_);

  double dist = ConvexPolygonPairDistance(vehicle, obstacle_polygon_);

  Vector values(1);
  values(0) = safety_margin_ - dist;
  return values;
}

}  // namespace my_al_ilqr
