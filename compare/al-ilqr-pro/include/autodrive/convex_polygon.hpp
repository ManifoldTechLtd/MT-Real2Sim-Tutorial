#pragma once

#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

class ConvexPolygon {
 public:
  explicit ConvexPolygon(std::vector<Vector> vertices);

  const std::vector<Vector>& Vertices() const { return vertices_; }
  int VertexCount() const { return static_cast<int>(vertices_.size()); }

  const std::vector<Vector>& OutwardNormals() const { return outward_normals_; }
  const std::vector<double>& Offsets() const { return offsets_; }
  int FaceCount() const { return static_cast<int>(outward_normals_.size()); }

  // Log-Sum-Exp smooth approximation of max_i(n_i · p - b_i).
  // Positive when point is outside, negative when inside.
  double LogSumExpDistance(const Vector& point, double alpha) const;

  // Gradient of LogSumExpDistance w.r.t. point: ∂LSE/∂p = Σ wᵢ·nᵢ
  Vector LogSumExpGradient(const Vector& point, double alpha) const;

  // Exact signed distance to boundary (Euclidean).
  // Positive outside, negative inside.
  double SignedDistance(const Vector& point) const;

 private:
  std::vector<Vector> vertices_;
  std::vector<Vector> outward_normals_;
  std::vector<double> offsets_;
};

// Signed distance between two convex polygons.
// Positive when separated, negative when penetrating.
double ConvexPolygonPairDistance(const ConvexPolygon& a, const ConvexPolygon& b);

// Axis-aligned or rotated rectangle as a ConvexPolygon.
ConvexPolygon MakeRectangle(double cx, double cy, double half_length,
                            double half_width, double yaw);

// Vehicle body polygon from rear-axle-frame state (x, y, yaw).
ConvexPolygon MakeVehiclePolygon(double x, double y, double yaw,
                                 double length, double width,
                                 double rear_axle_to_rear);

}  // namespace my_al_ilqr
