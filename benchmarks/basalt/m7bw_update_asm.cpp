#include <Eigen/Dense>

__attribute__((noinline)) void m7bw_apply(Eigen::AffineCompact2f& transform,
                                          const Eigen::Matrix3f& update) {
  transform *= update;
}
