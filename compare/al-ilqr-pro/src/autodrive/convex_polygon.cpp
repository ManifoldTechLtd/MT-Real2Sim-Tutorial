#include "autodrive/convex_polygon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

double PointToSegmentDistanceSq(const Vector& p, const Vector& a,
                                const Vector& b) {
  Vector ab = b - a;
  Vector ap = p - a;
  double t = ab.dot(ap) / ab.squaredNorm();
  t = std::clamp(t, 0.0, 1.0);
  Vector closest = a + t * ab;
  return (p - closest).squaredNorm();
}

}  // namespace

ConvexPolygon::ConvexPolygon(std::vector<Vector> vertices)
    : vertices_(std::move(vertices)) {
  const int n = static_cast<int>(vertices_.size());
  if (n < 3) {
    throw std::invalid_argument("ConvexPolygon requires at least 3 vertices.");
  }
  for (const auto& v : vertices_) {
    if (v.size() != 2) {
      throw std::invalid_argument("ConvexPolygon vertices must be 2D.");
    }
  }

  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const int k = (i + 2) % n;
    Vector e1 = vertices_[j] - vertices_[i];
    Vector e2 = vertices_[k] - vertices_[j];
    double cross = e1(0) * e2(1) - e1(1) * e2(0);
    if (cross < -1e-10) {
      throw std::invalid_argument(
          "ConvexPolygon vertices must be convex and in counter-clockwise "
          "order.");
    }
  }

  outward_normals_.reserve(n);
  offsets_.reserve(n);
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    Vector edge = vertices_[j] - vertices_[i];
    Vector normal(2);
    normal(0) = edge(1);
    normal(1) = -edge(0);
    double len = normal.norm();
    if (len < 1e-12) {
      throw std::invalid_argument("ConvexPolygon has degenerate edge.");
    }
    normal /= len;
    outward_normals_.push_back(normal);
    offsets_.push_back(normal.dot(vertices_[i]));
  }
}

double ConvexPolygon::LogSumExpDistance(const Vector& point,
                                       double alpha) const {
  double max_val = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    max_val = std::max(max_val, val);
  }

  double sum_exp = 0.0;
  for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    sum_exp += std::exp(alpha * (val - max_val));
  }

  return max_val + std::log(sum_exp) / alpha;
}

Vector ConvexPolygon::LogSumExpGradient(const Vector& point, double alpha) const {
  // Compute softmax weights
  double max_val = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    max_val = std::max(max_val, val);
  }

  double sum_exp = 0.0;
  for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    sum_exp += std::exp(alpha * (val - max_val));
  }

  // ∂LSE/∂p = Σ wᵢ · nᵢ
  Vector grad = Vector::Zero(2);
  for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    double w = std::exp(alpha * (val - max_val)) / sum_exp;
    grad += w * outward_normals_[i];
  }
  return grad;
}

double ConvexPolygon::SignedDistance(const Vector& point) const {
  bool inside = true;
  for (int i = 0; i < FaceCount(); ++i) {
    if (outward_normals_[i].dot(point) - offsets_[i] > 0.0) {
      inside = false;
      break;
    }
  }

  double min_dist_sq = std::numeric_limits<double>::infinity();
  const int n = VertexCount();
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    double d = PointToSegmentDistanceSq(point, vertices_[i], vertices_[j]);
    min_dist_sq = std::min(min_dist_sq, d);
  }

  double dist = std::sqrt(min_dist_sq);
  return inside ? -dist : dist;
}

double ConvexPolygonPairDistance(const ConvexPolygon& a,
                                const ConvexPolygon& b) {
  // SAT (Separating Axis Theorem) determines overlap / separation and gives
  // the exact penetration depth for convex polygons.
  bool separated = false;
  double min_overlap = std::numeric_limits<double>::infinity();

  auto testAxes = [&](const ConvexPolygon& poly) {
    for (int i = 0; i < poly.FaceCount(); ++i) {
      const Vector& axis = poly.OutwardNormals()[i];
      double min_a = std::numeric_limits<double>::infinity();
      double max_a = -std::numeric_limits<double>::infinity();
      for (const auto& v : a.Vertices()) {
        double proj = axis.dot(v);
        min_a = std::min(min_a, proj);
        max_a = std::max(max_a, proj);
      }
      double min_b = std::numeric_limits<double>::infinity();
      double max_b = -std::numeric_limits<double>::infinity();
      for (const auto& v : b.Vertices()) {
        double proj = axis.dot(v);
        min_b = std::min(min_b, proj);
        max_b = std::max(max_b, proj);
      }
      double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
      if (overlap < 0.0) {
        separated = true;
      }
      min_overlap = std::min(min_overlap, overlap);
    }
  };

  testAxes(a);
  testAxes(b);

  if (!separated) {
    return -min_overlap;
  }

  // Exact Euclidean distance via vertex-to-edge enumeration.
  double min_dist_sq = std::numeric_limits<double>::infinity();

  for (const auto& v : a.Vertices()) {
    for (int i = 0; i < b.VertexCount(); ++i) {
      int j = (i + 1) % b.VertexCount();
      double d = PointToSegmentDistanceSq(v, b.Vertices()[i], b.Vertices()[j]);
      min_dist_sq = std::min(min_dist_sq, d);
    }
  }

  for (const auto& v : b.Vertices()) {
    for (int i = 0; i < a.VertexCount(); ++i) {
      int j = (i + 1) % a.VertexCount();
      double d = PointToSegmentDistanceSq(v, a.Vertices()[i], a.Vertices()[j]);
      min_dist_sq = std::min(min_dist_sq, d);
    }
  }

  return std::sqrt(min_dist_sq);
}

ConvexPolygon MakeRectangle(double cx, double cy, double half_length,
                            double half_width, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double body[][2] = {
      {-half_length, -half_width},
      {half_length, -half_width},
      {half_length, half_width},
      {-half_length, half_width},
  };
  std::vector<Vector> vertices;
  vertices.reserve(4);
  for (const auto& corner : body) {
    Vector v(2);
    v(0) = cx + c * corner[0] - s * corner[1];
    v(1) = cy + s * corner[0] + c * corner[1];
    vertices.push_back(v);
  }
  return ConvexPolygon(std::move(vertices));
}

ConvexPolygon MakeVehiclePolygon(double x, double y, double yaw, double length,
                                 double width, double rear_axle_to_rear) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hw = 0.5 * width;
  const double body[][2] = {
      {-rear_axle_to_rear, -hw},
      {length - rear_axle_to_rear, -hw},
      {length - rear_axle_to_rear, hw},
      {-rear_axle_to_rear, hw},
  };
  std::vector<Vector> vertices;
  vertices.reserve(4);
  for (const auto& corner : body) {
    Vector v(2);
    v(0) = x + c * corner[0] - s * corner[1];
    v(1) = y + s * corner[0] + c * corner[1];
    vertices.push_back(v);
  }
  return ConvexPolygon(std::move(vertices));
}

}  // namespace my_al_ilqr
