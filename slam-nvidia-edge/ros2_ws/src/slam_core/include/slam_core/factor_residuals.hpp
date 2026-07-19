#pragma once

// Factor residuals for the sliding-window / fusion back-ends.
// Residual layouts are final; robust losses and solver wiring are the
// extension points (Ceres/GTSAM evaluators consume these directly).

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_core/imu_preintegration.hpp"
#include "slam_core/lie_group.hpp"

namespace slam_core {

// World-frame navigation state at a keyframe.
struct NavState {
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};  // R_wb
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // p_wb
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};      // v_w
  ImuBias bias;
};

// 15-dof IMU preintegration factor residual:
// [dR (3), dv (3), dp (3), dbg (3), dba (3)].
struct ImuFactorResidual {
  static constexpr int kDim = 15;

  static Eigen::Matrix<double, kDim, 1> evaluate(const Eigen::Matrix3d& delta_r_measured,
                                                 const Eigen::Matrix3d& delta_r_predicted,
                                                 const Eigen::Vector3d& delta_v_error,
                                                 const Eigen::Vector3d& delta_p_error,
                                                 const Eigen::Vector3d& gyro_bias_delta,
                                                 const Eigen::Vector3d& accel_bias_delta) {
    Eigen::Matrix<double, kDim, 1> r;
    r.segment<3>(0) = logSO3(delta_r_measured.transpose() * delta_r_predicted);
    r.segment<3>(3) = delta_v_error;
    r.segment<3>(6) = delta_p_error;
    r.segment<3>(9) = gyro_bias_delta;
    r.segment<3>(12) = accel_bias_delta;
    return r;
  }

  // Full between-states residual (Forster eq. 45): compares the preintegrated
  // deltas — first-order corrected to state_i's bias — against the deltas
  // implied by the two states under gravity.
  static Eigen::Matrix<double, kDim, 1> evaluate(const NavState& state_i, const NavState& state_j,
                                                 const ImuPreintegrator& pre,
                                                 const Eigen::Vector3d& gravity) {
    Eigen::Matrix3d delta_r;
    Eigen::Vector3d delta_v, delta_p;
    pre.biasCorrectedDelta(state_i.bias, delta_r, delta_v, delta_p);

    const double t = pre.deltaTime();
    const Eigen::Matrix3d Ri_t = state_i.rotation.transpose();
    const Eigen::Matrix3d delta_r_predicted = Ri_t * state_j.rotation;
    const Eigen::Vector3d delta_v_predicted =
        Ri_t * (state_j.velocity - state_i.velocity - gravity * t);
    const Eigen::Vector3d delta_p_predicted =
        Ri_t * (state_j.position - state_i.position - state_i.velocity * t - 0.5 * gravity * t * t);

    return evaluate(delta_r, delta_r_predicted, delta_v_predicted - delta_v,
                    delta_p_predicted - delta_p, state_j.bias.gyro - state_i.bias.gyro,
                    state_j.bias.accel - state_i.bias.accel);
  }
};

// 2-dof visual reprojection residual (pixel error).
struct ReprojectionFactorResidual {
  static constexpr int kDim = 2;

  // Pinhole projection of a camera-frame landmark; returns predicted - observed.
  // Landmarks at/behind the image plane yield a large fixed residual so the
  // robust loss (extension point) can down-weight them deterministically.
  static Eigen::Vector2d evaluate(const Eigen::Vector3d& landmark_camera, const Eigen::Vector2d& observed_px,
                                  double fx, double fy, double cx, double cy) {
    if (landmark_camera.z() <= 1e-6) {
      return Eigen::Vector2d::Constant(1e6);
    }
    const Eigen::Vector2d predicted(fx * landmark_camera.x() / landmark_camera.z() + cx,
                                    fy * landmark_camera.y() / landmark_camera.z() + cy);
    return predicted - observed_px;
  }

  // d(residual)/d(landmark_camera); zero for the clamped behind-plane branch.
  static Eigen::Matrix<double, 2, 3> jacobian(const Eigen::Vector3d& landmark_camera, double fx, double fy) {
    if (landmark_camera.z() <= 1e-6) {
      return Eigen::Matrix<double, 2, 3>::Zero();
    }
    const double inv_z = 1.0 / landmark_camera.z();
    Eigen::Matrix<double, 2, 3> J;
    J << fx * inv_z, 0.0, -fx * landmark_camera.x() * inv_z * inv_z,
        0.0, fy * inv_z, -fy * landmark_camera.y() * inv_z * inv_z;
    return J;
  }
};

// 1-dof LiDAR point-to-plane residual.
struct PointToPlaneFactorResidual {
  static constexpr int kDim = 1;

  static double evaluate(const Eigen::Vector3d& point_world, const Eigen::Vector3d& plane_unit_normal,
                         double plane_d) {
    return plane_unit_normal.dot(point_world) + plane_d;
  }
};

}  // namespace slam_core
