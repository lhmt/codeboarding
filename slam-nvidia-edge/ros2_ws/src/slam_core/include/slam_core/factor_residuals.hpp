#pragma once

// Factor residual shapes for the sliding-window / fusion back-ends.
// These are dimension-correct placeholders: the residual layouts are final,
// the error models are the extension points (swap in Ceres/GTSAM evaluators).

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_core/lie_group.hpp"

namespace slam_core {

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
