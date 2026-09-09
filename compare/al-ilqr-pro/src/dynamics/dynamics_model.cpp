#include "dynamics/dynamics_model.hpp"

#include <stdexcept>

namespace my_al_ilqr {

Matrix DynamicsModel::JacobianState(const Vector& /*state*/, const Vector& /*control*/,
                                    double /*dt*/) const {
  throw std::logic_error("DynamicsModel::JacobianState not implemented for this model.");
}

Matrix DynamicsModel::JacobianControl(const Vector& /*state*/, const Vector& /*control*/,
                                      double /*dt*/) const {
  throw std::logic_error("DynamicsModel::JacobianControl not implemented for this model.");
}

}  // namespace my_al_ilqr
