#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "autodrive/convex_polygon.hpp"
#include "autodrive/lse_polygon_obstacle_constraint.hpp"
#include "autodrive/polygon_collision_constraint.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"

using namespace my_al_ilqr;

namespace {

void AssertNear(double actual, double expected, double tol,
                const char* label) {
  if (std::abs(actual - expected) > tol) {
    std::cerr << "FAIL [" << label << "]: expected " << expected << ", got "
              << actual << std::endl;
    assert(false);
  }
}

ConvexPolygon MakeUnitSquare() {
  std::vector<Vector> v(4);
  v[0] = Vector(2); v[0] << -1.0, -1.0;
  v[1] = Vector(2); v[1] <<  1.0, -1.0;
  v[2] = Vector(2); v[2] <<  1.0,  1.0;
  v[3] = Vector(2); v[3] << -1.0,  1.0;
  return ConvexPolygon(std::move(v));
}

void TestConstruction() {
  auto poly = MakeUnitSquare();
  assert(poly.VertexCount() == 4);
  assert(poly.FaceCount() == 4);

  const auto& normals = poly.OutwardNormals();
  AssertNear(normals[0](0),  0.0, 1e-12, "n0.x");
  AssertNear(normals[0](1), -1.0, 1e-12, "n0.y");  // bottom face
  AssertNear(normals[1](0),  1.0, 1e-12, "n1.x");  // right face
  AssertNear(normals[2](0),  0.0, 1e-12, "n2.x");
  AssertNear(normals[2](1),  1.0, 1e-12, "n2.y");  // top face
  AssertNear(normals[3](0), -1.0, 1e-12, "n3.x");  // left face
  std::cout << "  Construction: PASSED" << std::endl;
}

void TestLSEOutside() {
  auto poly = MakeUnitSquare();
  Vector p(2); p << 3.0, 0.0;
  double lse = poly.LogSumExpDistance(p, 20.0);
  // Right face: n=(1,0), offset=1, val = 3-1 = 2.  LSE ≈ 2.
  AssertNear(lse, 2.0, 0.15, "LSE outside");
  assert(lse > 0.0);
  std::cout << "  LSE outside: PASSED (lse=" << lse << ")" << std::endl;
}

void TestLSEInside() {
  auto poly = MakeUnitSquare();
  Vector p(2); p << 0.0, 0.0;
  double lse = poly.LogSumExpDistance(p, 20.0);
  // All face values = -1.  LSE ≈ -1 + log(4)/20 ≈ -0.93.
  assert(lse < 0.0);
  std::cout << "  LSE inside: PASSED (lse=" << lse << ")" << std::endl;
}

void TestSDFOutside() {
  auto poly = MakeUnitSquare();
  Vector p(2); p << 3.0, 0.0;
  double sd = poly.SignedDistance(p);
  AssertNear(sd, 2.0, 1e-10, "SDF outside face");
  std::cout << "  SDF outside face: PASSED" << std::endl;
}

void TestSDFOutsideCorner() {
  auto poly = MakeUnitSquare();
  Vector p(2); p << 2.0, 2.0;
  double sd = poly.SignedDistance(p);
  AssertNear(sd, std::sqrt(2.0), 1e-10, "SDF outside corner");
  std::cout << "  SDF outside corner: PASSED" << std::endl;
}

void TestSDFInside() {
  auto poly = MakeUnitSquare();
  Vector p(2); p << 0.5, 0.0;
  double sd = poly.SignedDistance(p);
  AssertNear(sd, -0.5, 1e-10, "SDF inside");
  std::cout << "  SDF inside: PASSED" << std::endl;
}

void TestPairSeparated() {
  auto a = MakeUnitSquare();
  auto b = MakeRectangle(5.0, 0.0, 1.0, 1.0, 0.0);
  // a: [-1,1]×[-1,1],  b: [4,6]×[-1,1].  Gap = 3.
  double d = ConvexPolygonPairDistance(a, b);
  AssertNear(d, 3.0, 1e-10, "pair separated");
  std::cout << "  Pair separated: PASSED" << std::endl;
}

void TestPairPenetrating() {
  auto a = MakeUnitSquare();
  auto b = MakeRectangle(1.5, 0.0, 1.0, 1.0, 0.0);
  // a: [-1,1]×[-1,1], b: [0.5,2.5]×[-1,1].  x-overlap = 0.5.
  double d = ConvexPolygonPairDistance(a, b);
  assert(d < 0.0);
  AssertNear(d, -0.5, 1e-10, "pair penetrating");
  std::cout << "  Pair penetrating: PASSED" << std::endl;
}

void TestMakeRectangle() {
  auto r = MakeRectangle(2.0, 0.0, 1.5, 0.5, 0.0);
  assert(r.VertexCount() == 4);
  Vector p(2); p << 0.0, 0.0;
  double sd = r.SignedDistance(p);
  AssertNear(sd, 0.5, 1e-10, "rect dist");
  std::cout << "  MakeRectangle: PASSED" << std::endl;
}

void TestMakeVehiclePolygon() {
  auto vp = MakeVehiclePolygon(0.0, 0.0, 0.0, 4.6, 1.9, 1.0);
  assert(vp.VertexCount() == 4);
  // rear-right corner should be at (-1, -0.95)
  AssertNear(vp.Vertices()[0](0), -1.0, 1e-10, "vp rear x");
  AssertNear(vp.Vertices()[0](1), -0.95, 1e-10, "vp rear y");
  std::cout << "  MakeVehiclePolygon: PASSED" << std::endl;
}

void TestLSEConstraintSatisfied() {
  VehicleBodyConfig body;
  body.length = 4.6;
  body.width = 1.9;
  body.rear_axle_to_rear = 1.0;
  body.collision_circle_count = 2;
  VehicleCircleApproximation vehicle(body);
  auto obstacle = MakeRectangle(10.0, 0.0, 1.0, 0.5, 0.0);

  LSEPolygonObstacleConstraint constraint(vehicle, obstacle, 20.0);
  Vector state(4); state << 0.0, 0.0, 0.0, 2.0;
  Vector ctrl(2); ctrl << 0.0, 0.0;
  Vector c = constraint.Evaluate(state, ctrl);
  assert(c.size() == 2);
  for (int i = 0; i < c.size(); ++i) {
    assert(c(i) < 0.0);
  }
  std::cout << "  LSE constraint (separated): PASSED" << std::endl;
}

void TestLSEConstraintViolated() {
  VehicleBodyConfig body;
  body.length = 4.6;
  body.width = 1.9;
  body.rear_axle_to_rear = 1.0;
  body.collision_circle_count = 2;
  VehicleCircleApproximation vehicle(body);
  auto obstacle = MakeRectangle(1.0, 0.0, 1.0, 1.0, 0.0);

  LSEPolygonObstacleConstraint constraint(vehicle, obstacle, 20.0);
  Vector state(4); state << 0.0, 0.0, 0.0, 2.0;
  Vector ctrl(2); ctrl << 0.0, 0.0;
  Vector c = constraint.Evaluate(state, ctrl);
  bool any_violated = false;
  for (int i = 0; i < c.size(); ++i) {
    if (c(i) > 0.0) any_violated = true;
  }
  assert(any_violated);
  std::cout << "  LSE constraint (violated): PASSED" << std::endl;
}

void TestPolygonCollisionSatisfied() {
  auto obstacle = MakeRectangle(10.0, 0.0, 1.0, 0.5, 0.0);
  PolygonCollisionConstraint constraint(4.6, 1.9, 1.0, obstacle, 0.1);
  Vector state(4); state << 0.0, 0.0, 0.0, 2.0;
  Vector ctrl(2); ctrl << 0.0, 0.0;
  Vector c = constraint.Evaluate(state, ctrl);
  assert(c.size() == 1);
  assert(c(0) < 0.0);
  std::cout << "  Polygon collision (separated): PASSED" << std::endl;
}

void TestPolygonCollisionViolated() {
  auto obstacle = MakeRectangle(2.0, 0.0, 1.0, 1.0, 0.0);
  PolygonCollisionConstraint constraint(4.6, 1.9, 1.0, obstacle, 0.1);
  Vector state(4); state << 0.0, 0.0, 0.0, 2.0;
  Vector ctrl(2); ctrl << 0.0, 0.0;
  // Vehicle spans [-1, 3.6] × [-0.95, 0.95], obstacle [1,3] × [-1,1].
  // They overlap, so dist < 0, constraint > 0.
  Vector c = constraint.Evaluate(state, ctrl);
  assert(c(0) > 0.0);
  std::cout << "  Polygon collision (violated): PASSED" << std::endl;
}

void TestRotatedObstacle() {
  auto obstacle = MakeRectangle(6.0, 0.0, 2.0, 0.5, M_PI / 6.0);
  assert(obstacle.VertexCount() == 4);
  PolygonCollisionConstraint constraint(4.6, 1.9, 1.0, obstacle, 0.1);
  Vector state(4); state << 0.0, 0.0, 0.0, 2.0;
  Vector ctrl(2); ctrl << 0.0, 0.0;
  Vector c = constraint.Evaluate(state, ctrl);
  assert(c(0) < 0.0);
  std::cout << "  Rotated obstacle: PASSED" << std::endl;
}

}  // namespace

int main() {
  std::cout << "Polygon obstacle tests:" << std::endl;
  TestConstruction();
  TestLSEOutside();
  TestLSEInside();
  TestSDFOutside();
  TestSDFOutsideCorner();
  TestSDFInside();
  TestPairSeparated();
  TestPairPenetrating();
  TestMakeRectangle();
  TestMakeVehiclePolygon();
  TestLSEConstraintSatisfied();
  TestLSEConstraintViolated();
  TestPolygonCollisionSatisfied();
  TestPolygonCollisionViolated();
  TestRotatedObstacle();
  std::cout << "\nAll polygon obstacle tests PASSED." << std::endl;
  return 0;
}
