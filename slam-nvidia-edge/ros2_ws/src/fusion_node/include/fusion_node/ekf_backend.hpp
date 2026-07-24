#pragma once

// Error-state EKF fusion backend.
// Implements FusionBackend (fusion_backend.hpp) by holding persistent nominal
// state + covariance as member data across fuse() calls, since the interface
// itself is stateless-per-call. Error state order is [dp(3), dv(3), dtheta(3)].

#include <algorithm>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion_node/fusion_backend.hpp"
#include "slam_core/lie_group.hpp"

namespace fusion_node {

class ErrorStateEkfBackend final : public FusionBackend {
 public:
  std::optional<FusedState> fuse(const std::vector<SourceState>& sources) override {
    std::vector<const SourceState*> alive;
    alive.reserve(sources.size());
    for (const auto& s : sources) {
      if (s.weight > 0.0) alive.push_back(&s);
    }
    if (alive.empty()) return std::nullopt;

    if (!initialized_) {
      const SourceState* best = alive.front();
      for (const SourceState* s : alive) {
        if (s->weight > best->weight) best = s;
      }
      p_ = best->position;
      v_ = best->velocity;
      if (best->has_orientation) q_ = best->orientation;
      q_.normalize();
      P_ = 1.0 * Eigen::Matrix<double, 9, 9>::Identity();
      initialized_ = true;
    }

    // EXTENSION POINT: a real predict would use dt and a constant-velocity
    // model (p += v*dt); dt is not available through this stateless interface.
    P_ += Q_;

    FusedState out;
    for (const SourceState* s : alive) {
      const double r_scale = base_meas_var_ / std::max(s->weight, 1e-6);

      Eigen::Matrix<double, 3, 9> H_pos = Eigen::Matrix<double, 3, 9>::Zero();
      H_pos.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      update3(H_pos, s->position - p_, r_scale);

      Eigen::Matrix<double, 3, 9> H_vel = Eigen::Matrix<double, 3, 9>::Zero();
      H_vel.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
      update3(H_vel, s->velocity - v_, r_scale);

      if (s->has_orientation) {
        Eigen::Matrix<double, 3, 9> H_rot = Eigen::Matrix<double, 3, 9>::Zero();
        H_rot.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
        // World-frame residual Log(q_meas * q_nominal^-1): the error state is
        // the left perturbation true = Exp(dtheta) * nominal, so the injection
        // below (Exp(dx) * q_) must use the world-frame residual, not the body
        // one, or the update pushes along a rotated direction.
        const Eigen::Vector3d residual =
            slam_core::logSO3((s->orientation * q_.conjugate()).toRotationMatrix());
        update3(H_rot, residual, r_scale);
      }

      out.timestamp = std::max(out.timestamp, s->timestamp);
      ++out.num_sources;
    }

    out.position = p_;
    out.orientation = q_.normalized();
    out.velocity = v_;
    return out;
  }

  // Clears accumulated state back to the uninformed prior.
  void reset() {
    initialized_ = false;
    P_ = 100.0 * Eigen::Matrix<double, 9, 9>::Identity();
  }

  const Eigen::Matrix<double, 9, 9>& covariance() const { return P_; }

 private:
  // Sequential Kalman update for a 3-row measurement; injects the resulting
  // error-state delta into the nominal state and resets the error state.
  void update3(const Eigen::Matrix<double, 3, 9>& H, const Eigen::Vector3d& residual, double r_scale) {
    const Eigen::Matrix3d S = H * P_ * H.transpose() + r_scale * Eigen::Matrix3d::Identity();
    const Eigen::Matrix<double, 9, 3> K = P_ * H.transpose() * S.inverse();
    const Eigen::Matrix<double, 9, 1> dx = K * residual;

    p_ += dx.segment<3>(0);
    v_ += dx.segment<3>(3);
    q_ = Eigen::Quaterniond(slam_core::expSO3(dx.segment<3>(6))) * q_;
    q_.normalize();

    const Eigen::Matrix<double, 9, 9> I9 = Eigen::Matrix<double, 9, 9>::Identity();
    P_ = (I9 - K * H) * P_;
    P_ = 0.5 * (P_ + P_.transpose());
  }

  Eigen::Vector3d p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};

  // Covariance over [dp, dv, dtheta]; large diagonal until the first fuse().
  Eigen::Matrix<double, 9, 9> P_{100.0 * Eigen::Matrix<double, 9, 9>::Identity()};
  bool initialized_{false};

  // Process-noise inflation applied once per fuse() call (no dt available).
  Eigen::Matrix<double, 9, 9> Q_{[] {
    Eigen::Matrix<double, 9, 9> q = Eigen::Matrix<double, 9, 9>::Zero();
    q.diagonal() << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3;
    return q;
  }()};

  // Base measurement variance at weight == 1; scaled down by source weight.
  double base_meas_var_{0.01};
};

}  // namespace fusion_node
