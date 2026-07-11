#include "slam_core/health_rules.hpp"

#include "test_framework.hpp"

using slam_core::evaluateHealth;
using slam_core::HealthSample;
using slam_core::HealthThresholds;

static HealthSample nominal() {
  HealthSample s;
  s.tracked_features = 120;
  s.reprojection_error_median = 0.8;
  s.gyro_bias_norm = 0.01;
  s.accel_bias_norm = 0.1;
  s.condition_number = 1e4;
  return s;
}

static void run_tests() {
  CHECK(evaluateHealth(nominal()).status() == "OK");
  CHECK(evaluateHealth(nominal()).ok());

  {
    auto s = nominal();
    s.tracked_features = 29;
    CHECK(evaluateHealth(s).status() == "WARN_TRACKING");
    s.tracked_features = 30;  // boundary: rule is strictly less-than
    CHECK(evaluateHealth(s).status() == "OK");
  }
  {
    auto s = nominal();
    s.reprojection_error_median = 3.01;
    CHECK(evaluateHealth(s).status() == "WARN_REPROJECTION");
    s.reprojection_error_median = 3.0;  // boundary: rule is strictly greater-than
    CHECK(evaluateHealth(s).status() == "OK");
  }
  {
    auto s = nominal();
    s.gyro_bias_norm = 0.11;
    CHECK(evaluateHealth(s).status() == "WARN_IMU_GYRO");
  }
  {
    auto s = nominal();
    s.accel_bias_norm = 1.5;
    CHECK(evaluateHealth(s).status() == "WARN_IMU_ACCEL");
  }
  {
    auto s = nominal();
    s.condition_number = 1e11;
    const auto v = evaluateHealth(s);
    CHECK(v.status() == "WARN_DEGENERACY");
    CHECK(v.is_degenerate);
  }
  {
    // Multiple warnings joined in deterministic rule order
    auto s = nominal();
    s.tracked_features = 5;
    s.reprojection_error_median = 10.0;
    CHECK(evaluateHealth(s).status() == "WARN_TRACKING,WARN_REPROJECTION");
  }
  {
    // LOST dominates everything else
    auto s = nominal();
    s.tracked_features = 0;
    s.is_lost = true;
    const auto v = evaluateHealth(s);
    CHECK(v.status() == "LOST");
    CHECK(v.is_lost);
    CHECK(v.warnings.empty());
  }
  {
    // Custom thresholds are honored
    HealthThresholds t;
    t.min_tracked_features = 100;
    auto s = nominal();  // 120 features
    CHECK(evaluateHealth(s, t).status() == "OK");
    s.tracked_features = 99;
    CHECK(evaluateHealth(s, t).status() == "WARN_TRACKING");
  }
}

TEST_MAIN()
