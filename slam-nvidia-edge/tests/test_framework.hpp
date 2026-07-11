#pragma once

// Minimal assertion harness: zero dependencies so these tests build off-robot
// with plain CMake and on-robot under colcon without pulling in gtest.

#include <cmath>
#include <cstdio>

namespace slam_test {
inline int failures = 0;
}

#define CHECK(cond)                                                                   \
  do {                                                                                \
    if (!(cond)) {                                                                    \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                     \
      ++slam_test::failures;                                                          \
    }                                                                                 \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                         \
  do {                                                                                \
    const double _a = (a), _b = (b);                                                  \
    if (!(std::fabs(_a - _b) <= (tol))) {                                             \
      std::printf("FAIL %s:%d: |%s - %s| = %g > %g\n", __FILE__, __LINE__, #a, #b,    \
                  std::fabs(_a - _b), static_cast<double>(tol));                      \
      ++slam_test::failures;                                                          \
    }                                                                                 \
  } while (0)

#define TEST_MAIN()                                                                   \
  int main() {                                                                        \
    run_tests();                                                                      \
    if (slam_test::failures == 0) std::printf("PASS\n");                              \
    return slam_test::failures == 0 ? 0 : 1;                                          \
  }
