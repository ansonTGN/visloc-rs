#include <Eigen/Core>
#ifndef NO_TRACE
#include <unsupported/Eigen/NonLinearOptimization>
#endif
#include <nlohmann/json.hpp>
#ifndef NO_TRACE
#include <opengv/OptimizationFunctor.hpp>
#include <opengv/math/cayley.hpp>
#endif
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#ifndef TRACE_ONLY
#include <opengv/relative_pose/methods.hpp>
#endif
#include <opengv/triangulation/methods.hpp>

#ifndef NO_TRACE
extern Eigen::Vector3d m8c_traced_triangulate(
    opengv::relative_pose::CentralRelativeAdapter &adapter, std::size_t index);
#endif

#include <fstream>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using json = nlohmann::json;

static std::uint64_t bits(double value) {
  std::uint64_t out = 0;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

static const json &feature(const json &raw, std::uint64_t frame, int cam) {
  for (const auto &item : raw.at("features")) {
    if (item.at("time_cam_id").at("frame_id") == frame &&
        item.at("time_cam_id").at("cam_id") == cam)
      return item;
  }
  throw std::runtime_error("feature not found");
}

static Eigen::Vector3d ray(const json &f, std::size_t i) {
  const auto &v = f.at("corners_3d").at(i);
  return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
}

#ifndef NO_TRACE
struct TraceFunctor : opengv::OptimizationFunctor<double> {
  opengv::relative_pose::CentralRelativeAdapter &adapter;
  TraceFunctor(opengv::relative_pose::CentralRelativeAdapter &a, int n)
      : opengv::OptimizationFunctor<double>(6, n), adapter(a) {}

  int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const {
    const Eigen::Vector3d translation = x.head<3>();
    const Eigen::Vector3d cayley = x.tail<3>();
    const Eigen::Matrix3d rotation = opengv::math::cayley2rot(cayley);
    for (int i = 0; i < values(); ++i) {
      adapter.sett12(translation);
      adapter.setR12(rotation);
      Eigen::Matrix<double, 4, 1> p_hom;
      p_hom(3) = 1.0;
      p_hom.template head<3>() =
          std::getenv("M8C_TRACE_SOURCE_TRI_CALL") != nullptr
          ? m8c_traced_triangulate(adapter, i)
              : opengv::triangulation::triangulate2(adapter, i);
      const Eigen::Matrix3d inverse_rotation = rotation.transpose();
      const Eigen::Vector3d inverse_translation = -inverse_rotation * translation;
      Eigen::Matrix<double, 3, 4> inverse_solution;
      inverse_solution.template block<3, 3>(0, 0) = inverse_rotation;
      inverse_solution.col(3) = inverse_translation;
      const Eigen::Vector3d point = p_hom.template head<3>();
      const Eigen::Vector3d second = inverse_solution * p_hom;
      const Eigen::Vector3d first_unit = point / point.norm();
      const Eigen::Vector3d second_unit = second / second.norm();
      static bool printed_target = false;
      if (!printed_target && translation[0] > 0.17 && translation[0] < 0.18 && i == 0) {
        Eigen::Matrix3d c1 = adapter.getR12() - Eigen::Matrix3d::Identity();
        Eigen::Matrix3d c2 = adapter.getR12() + Eigen::Matrix3d::Identity();
        Eigen::Matrix3d ci = c2.inverse();
        Eigen::Matrix3d cc = c1 * ci;
        std::cout << std::setprecision(17) << "TRACE_INTERMEDIATE cayley=" << cayley.transpose()
                  << " p=" << point.transpose()
                  << " second=" << second.transpose() << "\n";
        std::cout << "TRACE_CAYLEY c2inv\n" << ci << "\nTRACE_C " << cc << "\n";
        printed_target = true;
      }
      const Eigen::Vector3d f1 = adapter.getBearingVector1(i);
      const Eigen::Vector3d f2 = adapter.getBearingVector2(i);
      const int triang_trace_limit = std::getenv("M8C_TRACE_TRIANG_ALL") != nullptr ? values() : 16;
      if (std::getenv("M8C_TRACE_TRIANG") != nullptr &&
          translation[0] > 0.17 && translation[0] < 0.18 && i < triang_trace_limit) {
        const Eigen::Vector3d right_unrotated = rotation * f2;
        const double a00 = f1.dot(f1);
        const double a10 = f1.dot(right_unrotated);
        const double a01 = -a10;
        const double a11 = -right_unrotated.dot(right_unrotated);
        const double b0 = translation.dot(f1);
        const double b1 = translation.dot(right_unrotated);
        Eigen::Matrix2d A;
        A(0, 0) = a00;
        A(1, 0) = a10;
        A(0, 1) = a01;
        A(1, 1) = a11;
        const double det_manual = a00 * a11 - a10 * a01;
        const double det_eigen = A.determinant();
        const double invdet_manual = 1.0 / det_manual;
        const double invdet_eigen_expr = 1.0 / A.determinant();
        const double det_fma_left = std::fma(a00, a11, -(a10 * a01));
        const double det_fma_right = std::fma(-a10, a01, a00 * a11);
        const double inverse_direct00 = a11 / det_manual;
        const double inverse_direct01 = -a01 / det_manual;
        const double inverse_direct10 = -a10 / det_manual;
        const double inverse_direct11 = a00 / det_manual;
        const Eigen::Matrix2d inverse = A.inverse();
        const Eigen::Vector2d lambda = inverse * Eigen::Vector2d(b0, b1);
        const double lambda0_p0 = inverse(0, 0) * b0;
        const double lambda0_p1 = inverse(0, 1) * b1;
        const double lambda1_p0 = inverse(1, 0) * b0;
        const double lambda1_p1 = inverse(1, 1) * b1;
        const double lambda0_sum = lambda0_p0 + lambda0_p1;
        const double lambda1_sum = lambda1_p0 + lambda1_p1;
        const double lambda0_fma = std::fma(inverse(0, 1), b1, lambda0_p0);
        const double lambda1_fma = std::fma(inverse(1, 1), b1, lambda1_p0);
        const Eigen::Vector3d point_manual =
            (lambda[0] * f1 + translation + lambda[1] * right_unrotated) / 2.0;
        const Eigen::Vector3d xm_source = lambda[0] * f1;
        const Eigen::Vector3d xn_source = translation + lambda[1] * right_unrotated;
        const Eigen::Vector3d point_source = (xm_source + xn_source) / 2.0;
        std::cout << std::hex << "TRACE_TRIANG i=" << i
                  << " a=" << bits(a00) << ',' << bits(a10) << ',' << bits(a01)
                  << ',' << bits(a11) << " b=" << bits(b0) << ',' << bits(b1)
                  << " det=" << bits(det_manual) << ',' << bits(det_eigen)
                  << " inv=" << bits(inverse(0, 0)) << ',' << bits(inverse(0, 1)) << ','
                  << bits(inverse(1, 0)) << ',' << bits(inverse(1, 1))
                  << " id=" << bits(invdet_manual) << ',' << bits(inverse_direct00) << ','
                  << bits(inverse_direct01) << ',' << bits(inverse_direct10) << ','
                  << bits(inverse_direct11) << " ie=" << bits(invdet_eigen_expr)
                  << " ir=" << bits(1.0 / det_fma_left) << ',' << bits(1.0 / det_fma_right)
                  << " lp=" << bits(lambda0_p0) << ',' << bits(lambda0_p1) << ','
                  << bits(lambda1_p0) << ',' << bits(lambda1_p1)
                  << " ls=" << bits(lambda0_sum) << ',' << bits(lambda1_sum)
                  << " lf=" << bits(lambda0_fma) << ',' << bits(lambda1_fma)
                  << " lam=" << bits(lambda[0]) << ',' << bits(lambda[1])
                  << " pm=" << bits(point_manual[0]) << ',' << bits(point_manual[1])
                  << ',' << bits(point_manual[2]) << " po=" << bits(point[0]) << ','
                  << bits(point[1]) << ',' << bits(point[2])
                  << " xs=" << bits(xm_source[0]) << ',' << bits(xm_source[1]) << ','
                  << bits(xm_source[2]) << " ns=" << bits(xn_source[0]) << ','
                  << bits(xn_source[1]) << ',' << bits(xn_source[2]) << " ps="
                  << bits(point_source[0]) << ',' << bits(point_source[1]) << ','
                  << bits(point_source[2]) << std::dec << "\n";
      }
      fvec(i) = (1.0 - f1.dot(first_unit)) + (1.0 - f2.dot(second_unit));
      const int trace_limit = std::getenv("M8C_TRACE_BITS_ALL") != nullptr ? values() : 8;
      if (std::getenv("M8C_TRACE_BITS") != nullptr && i < trace_limit) {
        std::cout << std::hex << "TRACE_BITS i=" << i
                  << " p=" << bits(point[0]) << ',' << bits(point[1]) << ',' << bits(point[2])
                  << " s=" << bits(second[0]) << ',' << bits(second[1]) << ',' << bits(second[2])
                  << " it=" << bits(inverse_solution(0, 3)) << ','
                  << bits(inverse_solution(1, 3)) << ',' << bits(inverse_solution(2, 3))
                  << " ir=" << bits(inverse_solution(2, 0)) << ','
                  << bits(inverse_solution(2, 1)) << ',' << bits(inverse_solution(2, 2))
                  << " n=" << bits(point.norm()) << ',' << bits(second.norm())
                  << " d=" << bits(f1.dot(first_unit)) << ',' << bits(f2.dot(second_unit))
                  << " f=" << bits(fvec(i)) << std::dec << "\n";
      }
    }
    if (std::getenv("M8C_TRACE_PERT_BITS") != nullptr
        && x[0] > 0.1748 && x[0] < 0.1749
        && x[1] < -0.6289 && x[1] > -0.6291) {
      static std::size_t trace_seed7_calls = 0;
      if (trace_seed7_calls < 7) {
        int changed = -1;
        static Eigen::VectorXd trace_seed7_base;
        if (trace_seed7_calls == 0) trace_seed7_base = x;
        else for (int col = 0; col < 6; ++col)
          if (x[col] != trace_seed7_base[col]) { changed = col; break; }
        std::cout << std::hex << "TRACE_PERT_BITS call=" << trace_seed7_calls
                  << " col=" << changed << " bits=";
        for (int row = 0; row < values(); ++row) {
          if (row) std::cout << ',';
          std::cout << bits(fvec(row));
        }
        std::cout << std::dec << "\n";
      }
      ++trace_seed7_calls;
    }
    static bool printed_residuals = false;
    if (!printed_residuals && translation[0] > 0.17 && translation[0] < 0.18) {
      std::cout << std::setprecision(17) << "TRACE_INITIAL_X " << x.transpose()
                << " fnorm=" << fvec.stableNorm() << " f=" << fvec.transpose() << "\n";
      printed_residuals = true;
    }
    return 0;
  }
};
#endif

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::ifstream raw_file(argv[1]);
  std::ifstream oracle_file(argv[2]);
  json raw, oracle;
  raw_file >> raw;
  oracle_file >> oracle;
  const auto &entries = oracle.at("bow_query").at("seeded_temporal_oracle");
  const std::uint64_t left_frame = 1403636580113555456ULL;
  const std::uint64_t right_frame = 1403636579763555584ULL;
  const auto &lf = feature(raw, left_frame, 0);
  const auto &rf = feature(raw, right_frame, 1);
  for (const auto &entry : entries) {
    if (entry.at("left").at("frame_id") != left_frame ||
        entry.at("left").at("cam_id") != 0 ||
        entry.at("right").at("frame_id") != right_frame ||
        entry.at("right").at("cam_id") != 1)
      continue;
    opengv::bearingVectors_t b1, b2;
    for (const auto &pair : entry.at("ransac_inlier_ids")) {
      b1.push_back(ray(lf, pair.at(0)));
      b2.push_back(ray(rf, pair.at(1)));
    }
    opengv::relative_pose::CentralRelativeAdapter adapter(b1, b2);
    Eigen::Matrix<double, 3, 3> R;
    auto &jr = entry.at("ransac_model_rotation");
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R(r, c) = jr.at(r).at(c);
    Eigen::Vector3d t;
    auto &jt = entry.at("ransac_model_translation");
    for (int i = 0; i < 3; ++i) t(i) = jt.at(i);
    adapter.setR12(R);
    adapter.sett12(t);
#ifndef TRACE_ONLY
    const auto got = opengv::relative_pose::optimize_nonlinear(adapter);
#endif
    std::cout << std::setprecision(17) << "seed " << entry.at("seed") << " n " << b1.size() << "\n";
    std::cout << "initial_t " << t.transpose() << "\n";
    std::cout << "initial_R\n" << R << "\n";
#ifndef TRACE_ONLY
    std::cout << "got_t " << got.col(3).head(3).transpose() << "\n";
    std::cout << "got_R\n" << got.topLeftCorner<3,3>() << "\n";
#endif

#ifndef NO_TRACE
    adapter.setR12(R);
    adapter.sett12(t);
    Eigen::VectorXd x(6);
    x.head<3>() = t;
    x.tail<3>() = opengv::math::rot2cayley(R);
    TraceFunctor functor(adapter, static_cast<int>(b1.size()));
    Eigen::NumericalDiff<TraceFunctor> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<TraceFunctor>> lm(num_diff);
    if (entry.at("seed") == 7) {
      Eigen::MatrixXd j(b1.size(), 6);
      num_diff.df(x, j);
      Eigen::VectorXd cn = j.colwise().blueNorm();
      Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(j);
      Eigen::MatrixXd qrmat = qr.matrixQR();
      Eigen::VectorXd fv(b1.size());
      functor(x, fv);
      Eigen::VectorXd qtf = qr.householderQ().adjoint() * fv;
      Eigen::VectorXd diag = cn;
      double delta = (diag.cwiseProduct(x)).stableNorm() * 100.0;
      for (int col = 0; col < 6; ++col) {
        Eigen::VectorXd xp = x;
        double h = std::sqrt(std::numeric_limits<double>::epsilon()) * std::abs(x[col]);
        if (h == 0.0) h = std::sqrt(std::numeric_limits<double>::epsilon());
        xp[col] += h;
        Eigen::VectorXd fp(b1.size());
        functor(xp, fp);
        std::cout << std::setprecision(17) << "TRACE_PERT col=" << col << " h=" << h
                  << " xh=" << xp[col] << " f=" << fp.transpose() << "\n";
      }
      std::cout << std::setprecision(17) << "TRACE_QR colnorm=" << cn.transpose()
                << " piv=" << qr.colsPermutation().indices().transpose()
                << " delta=" << delta << " qtf=" << qtf.head(6).transpose() << "\n";
      std::cout << "TRACE_QR_R\n" << qrmat.topLeftCorner(6, 6) << "\n";
    }
    lm.resetParameters();
    lm.parameters.ftol = 1.E1 * Eigen::NumTraits<double>::epsilon();
    lm.parameters.xtol = 1.E1 * Eigen::NumTraits<double>::epsilon();
    lm.parameters.maxfev = 1000;
    auto status = lm.minimizeInit(x);
    std::cout << "trace_begin seed " << entry.at("seed") << " status=" << status << "\n";
    do {
      status = lm.minimizeOneStep(x);
      std::cout << "step status=" << status << " iter=" << lm.iter << " nfev=" << lm.nfev
                << " fnorm=" << lm.fnorm << " gnorm=" << lm.gnorm
                << " par=" << lm.lm_param() << " x=" << x.transpose() << "\n";
    } while (status == Eigen::LevenbergMarquardtSpace::Running);
#endif
  }
}
