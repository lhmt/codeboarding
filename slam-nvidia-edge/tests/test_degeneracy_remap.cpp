#include "lio_node/degeneracy_detector.hpp"

#include <Eigen/Eigenvalues>

#include "test_framework.hpp"

using lio_node::DegeneracyDetector;

// Build a symmetric 6x6 Hessian with a chosen eigen-spectrum in a rotated basis
// (so degenerate directions are not axis-aligned — exercises the projection).
static Eigen::Matrix<double, 6, 6> hessianWithSpectrum(const Eigen::Matrix<double, 6, 1>& eigenvalues,
                                                       Eigen::Matrix<double, 6, 6>& basis_out) {
  // Deterministic orthonormal basis from the QR of a fixed non-symmetric matrix.
  Eigen::Matrix<double, 6, 6> m;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) m(r, c) = std::sin(1.0 + r * 6 + c) + (r == c ? 3.0 : 0.0);
  const Eigen::HouseholderQR<Eigen::Matrix<double, 6, 6>> qr(m);
  const Eigen::Matrix<double, 6, 6> q = qr.householderQ();
  basis_out = q;
  return q * eigenvalues.asDiagonal() * q.transpose();
}

static void run_tests() {
  // Well-conditioned Hessian: projection is (near) identity, remap is a no-op.
  {
    DegeneracyDetector det(1e6);
    Eigen::Matrix<double, 6, 6> basis;
    Eigen::Matrix<double, 6, 1> ev;
    ev << 1000, 900, 800, 700, 600, 500;
    const auto H = hessianWithSpectrum(ev, basis);
    const auto P = det.observableProjection(H);
    CHECK((P - Eigen::Matrix<double, 6, 6>::Identity()).norm() < 1e-9);
    Eigen::Matrix<double, 6, 1> u;
    u << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
    CHECK((det.remap(u, H) - u).norm() < 1e-9);
    CHECK(det.analyze(H).num_degenerate_directions == 0);
  }

  // Two eigenvalues far below max/threshold => 2 degenerate directions.
  // The remapped update must have ~zero component along those two eigenvectors
  // and preserve its component along the strong ones.
  {
    DegeneracyDetector det(1e3);  // cutoff = max_ev / 1e3
    Eigen::Matrix<double, 6, 6> basis;
    Eigen::Matrix<double, 6, 1> ev;
    ev << 1.0e6, 8.0e5, 6.0e5, 4.0e5, 1.0e1, 5.0e0;  // last two below 1e6/1e3 = 1e3
    const auto H = hessianWithSpectrum(ev, basis);

    const auto report = det.analyze(H);
    CHECK(report.num_degenerate_directions == 2);
    CHECK(report.is_degenerate);

    Eigen::Matrix<double, 6, 1> u;
    u << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    const auto remapped = det.remap(u, H);

    // Eigenvectors are basis columns; SelfAdjointEigenSolver sorts ascending,
    // so basis columns 4 and 5 (largest ev) are strong, columns 0,1 (ev 5,10)
    // are the weak ones we built. Verify against the detector's own solver.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
    const auto& V = solver.eigenvectors();
    const auto& evs = solver.eigenvalues();
    const double cutoff = evs.maxCoeff() / 1e3;
    for (int i = 0; i < 6; ++i) {
      const double comp_remapped = V.col(i).dot(remapped);
      const double comp_original = V.col(i).dot(u);
      if (evs[i] < cutoff) {
        CHECK(std::abs(comp_remapped) < 1e-9);            // unobservable: zeroed
      } else {
        CHECK(std::abs(comp_remapped - comp_original) < 1e-9);  // observable: kept
      }
    }
    // Projection is idempotent and symmetric.
    const auto P = det.observableProjection(H);
    CHECK((P * P - P).norm() < 1e-9);
    CHECK((P - P.transpose()).norm() < 1e-12);
  }

  // Physical single-plane case: strong along one translation axis, ~zero along
  // the two in-plane axes; rotation fully observable. In-plane translation of
  // a bogus increment is removed, normal-axis translation and rotation kept.
  {
    DegeneracyDetector det(1e6);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    // [rot(3), trans(3)] order.
    H(0, 0) = 5e4;
    H(1, 1) = 5e4;
    H(2, 2) = 5e4;   // rotation observable
    H(3, 3) = 1e5;   // translation along x (plane normal) observable
    H(4, 4) = 1e-3;  // translation along y (in-plane) unobservable
    H(5, 5) = 1e-3;  // translation along z (in-plane) unobservable

    const auto report = det.analyze(H);
    CHECK(report.is_degenerate);
    CHECK(report.num_degenerate_directions == 2);

    Eigen::Matrix<double, 6, 1> u;
    u << 0.01, -0.02, 0.03, 0.5, 0.4, -0.3;  // rot + full translation increment
    const auto remapped = det.remap(u, H);
    CHECK((remapped.head<3>() - u.head<3>()).norm() < 1e-9);  // rotation preserved
    CHECK(std::abs(remapped(3) - 0.5) < 1e-9);                // normal translation kept
    CHECK(std::abs(remapped(4)) < 1e-9);                      // in-plane y removed
    CHECK(std::abs(remapped(5)) < 1e-9);                      // in-plane z removed
  }

  // Degenerate (non-positive max eigenvalue) => nothing observable, remap zeros.
  {
    DegeneracyDetector det(1e6);
    const Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> u;
    u << 1, 2, 3, 4, 5, 6;
    CHECK(det.remap(u, H).norm() < 1e-12);
  }
}

TEST_MAIN()
