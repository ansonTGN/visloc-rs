// Diagnostic-only f32 prediction velocity boundary probe for pinned Basalt.
//
// This intentionally calls IntegratedImuMeasurement<float>::predictState with
// the exact MH_01 frame-2 state and the (frame-2, frame-3] IMU interval by
// default.  Passing `frame3` selects the exact frame-3 state and (frame-3,
// frame-4] interval.  It prints the gravity product, Sophus point action,
// several scalar association candidates, and the direct header result so a
// one-ULP velocity difference can be attributed without changing the
// estimator.

#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/preintegration.h>

#include <cstdint>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

uint32_t bits(float value) {
  uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Derived>
void print_bits(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ",";
    std::cout << std::hex << bits(value.derived()(i));
  }
  std::cout << std::dec << "\n";
}

template <typename Derived>
void print_bits_scalar(const char* name,
                       const Eigen::MatrixBase<Derived>& value) {
  print_bits(name, value);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: m7aq_predict_velocity_probe <imu.csv> [frame3]\n";
    return 2;
  }

  const bool frame3 = argc == 3 && std::string(argv[2]) == "frame3";

  using Scalar = float;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using SO3 = Sophus::SO3<Scalar>;

  // Exact pre-solver frame-2 state from the pinned five-frame oracle.
  const int64_t start_ns = frame3 ? 1403636579913555456LL
                                  : 1403636579863555584LL;
  const int64_t end_ns = frame3 ? 1403636579963555584LL
                                : 1403636579913555456LL;
  const Quaternion q2 = frame3
                            ? Quaternion(Scalar(0.589211106300354),
                                         Scalar(-0.054311808198690414),
                                         Scalar(-0.80612975358963013),
                                         Scalar(-0.0059463228099048138))
                            : Quaternion(Scalar(0.59337842464447021),
                                         Scalar(-0.053945489227771759),
                                         Scalar(-0.80309677124023438),
                                         Scalar(-0.0052577960304915905));
  const Vec3 p2 = frame3
                      ? Vec3(Scalar(0.0043563395738601685),
                             Scalar(-0.0011303708888590336),
                             Scalar(-0.016291741281747818))
                      : Vec3(Scalar(0.0018608879763633013),
                             Scalar(-0.00048369081923738122),
                             Scalar(-0.0076972413808107376));
  const Vec3 v2 = frame3
                      ? Vec3(Scalar(0.064949572086334229),
                             Scalar(-0.018744025379419327),
                             Scalar(-0.19759386777877808))
                      : Vec3(Scalar(0.034741103649139404),
                             Scalar(-0.0089075751602649689),
                             Scalar(-0.14201188087463379));
  const Vec3 gravity(Scalar(0), Scalar(0), Scalar(-9.8100004196166992));

  basalt::CalibAccelBias<Scalar> accel_calib;
  Eigen::Matrix<Scalar, 9, 1> accel_param;
  accel_param << Scalar(-0.003025405479279035), Scalar(0.1200005286487319),
      Scalar(0.06708820471592454), Scalar(0), Scalar(0), Scalar(0),
      Scalar(0), Scalar(0), Scalar(0);
  accel_calib.getParam() = accel_param;
  basalt::CalibGyroBias<Scalar> gyro_calib;
  Eigen::Matrix<Scalar, 12, 1> gyro_param;
  gyro_param << Scalar(-0.002186848441668376), Scalar(0.020427823167917037),
      Scalar(0.07668367023977922), Scalar(0), Scalar(0), Scalar(0),
      Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0);
  gyro_calib.getParam() = gyro_param;

  const Vec3 accel_noise =
      Vec3::Constant(Scalar(0.016) * std::sqrt(Scalar(200)));
  const Vec3 gyro_noise =
      Vec3::Constant(Scalar(0.000282) * std::sqrt(Scalar(200)));
  const Vec3 accel_cov = accel_noise.array().square();
  const Vec3 gyro_cov = gyro_noise.array().square();

  basalt::IntegratedImuMeasurement<Scalar> measurement(
      start_ns, Vec3::Zero(), Vec3::Zero());
  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "cannot open IMU CSV\n";
    return 3;
  }
  std::string line;
  int64_t previous_data_t_ns = start_ns;
  SO3 manual_rotation;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    for (char& c : line) {
      if (c == ',') c = ' ';
    }
    std::stringstream row(line);
    int64_t t_ns = 0;
    double wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
    if (!(row >> t_ns >> wx >> wy >> wz >> ax >> ay >> az)) return 4;
    if (t_ns <= start_ns || t_ns > end_ns) continue;
    basalt::ImuData<Scalar> data;
    data.t_ns = t_ns;
    data.gyro = gyro_calib.getCalibrated(
        Vec3(static_cast<Scalar>(wx), static_cast<Scalar>(wy),
             static_cast<Scalar>(wz)));
    data.accel = accel_calib.getCalibrated(
        Vec3(static_cast<Scalar>(ax), static_cast<Scalar>(ay),
             static_cast<Scalar>(az)));
    const SO3 manual_half =
        manual_rotation * SO3::exp(Scalar(0.5) *
                                   static_cast<Scalar>(t_ns - previous_data_t_ns) *
                                   Scalar(1e-9) * data.gyro);
    const Vec3 manual_accel_world = manual_half.matrix() * data.accel;
    const auto manual_matrix = manual_half.matrix();
    const auto dot_left = [](Scalar a, Scalar b, Scalar c, Scalar d,
                             Scalar e, Scalar f) {
      return std::fma(a, b, c * d) + e * f;
    };
    const auto dot_right = [](Scalar a, Scalar b, Scalar c, Scalar d,
                              Scalar e, Scalar f) {
      return a * b + std::fma(c, d, e * f);
    };
    const auto dot_nested = [](Scalar a, Scalar b, Scalar c, Scalar d,
                               Scalar e, Scalar f) {
      return std::fma(a, b, std::fma(c, d, e * f));
    };
    const auto dot_accum = [](Scalar a, Scalar b, Scalar c, Scalar d,
                              Scalar e, Scalar f) {
      return std::fma(e, f, std::fma(c, d, a * b));
    };
    const auto dot_plain_left = [](Scalar a, Scalar b, Scalar c, Scalar d,
                                   Scalar e, Scalar f) {
      return (a * b + c * d) + e * f;
    };
    const Vec3 aw_left(
        dot_left(manual_matrix(0, 0), data.accel.x(), manual_matrix(0, 1),
                 data.accel.y(), manual_matrix(0, 2), data.accel.z()),
        dot_left(manual_matrix(1, 0), data.accel.x(), manual_matrix(1, 1),
                 data.accel.y(), manual_matrix(1, 2), data.accel.z()),
        dot_left(manual_matrix(2, 0), data.accel.x(), manual_matrix(2, 1),
                 data.accel.y(), manual_matrix(2, 2), data.accel.z()));
    const Vec3 aw_right(
        dot_right(manual_matrix(0, 0), data.accel.x(), manual_matrix(0, 1),
                  data.accel.y(), manual_matrix(0, 2), data.accel.z()),
        dot_right(manual_matrix(1, 0), data.accel.x(), manual_matrix(1, 1),
                  data.accel.y(), manual_matrix(1, 2), data.accel.z()),
        dot_right(manual_matrix(2, 0), data.accel.x(), manual_matrix(2, 1),
                  data.accel.y(), manual_matrix(2, 2), data.accel.z()));
    const Vec3 aw_nested(
        dot_nested(manual_matrix(0, 0), data.accel.x(), manual_matrix(0, 1),
                   data.accel.y(), manual_matrix(0, 2), data.accel.z()),
        dot_nested(manual_matrix(1, 0), data.accel.x(), manual_matrix(1, 1),
                   data.accel.y(), manual_matrix(1, 2), data.accel.z()),
        dot_nested(manual_matrix(2, 0), data.accel.x(), manual_matrix(2, 1),
                   data.accel.y(), manual_matrix(2, 2), data.accel.z()));
    const Vec3 aw_accum(
        dot_accum(manual_matrix(0, 0), data.accel.x(), manual_matrix(0, 1),
                  data.accel.y(), manual_matrix(0, 2), data.accel.z()),
        dot_accum(manual_matrix(1, 0), data.accel.x(), manual_matrix(1, 1),
                  data.accel.y(), manual_matrix(1, 2), data.accel.z()),
        dot_accum(manual_matrix(2, 0), data.accel.x(), manual_matrix(2, 1),
                  data.accel.y(), manual_matrix(2, 2), data.accel.z()));
    const Vec3 aw_plain_left(
        dot_plain_left(manual_matrix(0, 0), data.accel.x(), manual_matrix(0, 1),
                       data.accel.y(), manual_matrix(0, 2), data.accel.z()),
        dot_plain_left(manual_matrix(1, 0), data.accel.x(), manual_matrix(1, 1),
                       data.accel.y(), manual_matrix(1, 2), data.accel.z()),
        dot_plain_left(manual_matrix(2, 0), data.accel.x(), manual_matrix(2, 1),
                       data.accel.y(), manual_matrix(2, 2), data.accel.z()));
    measurement.integrate(data, accel_cov, gyro_cov);
    // Keep the per-sample state visible while isolating an interval.  The
    // frame-3 O2/O3 comparison uses these records to locate the first packet
    // evaluator boundary before the final delta is consumed by predictState.
    const auto& step = measurement.getDeltaState();
    print_bits("step_aw", manual_accel_world);
    print_bits("step_aw_left", aw_left);
    print_bits("step_aw_right", aw_right);
    print_bits("step_aw_nested", aw_nested);
    print_bits("step_aw_accum", aw_accum);
    print_bits("step_aw_plain_left", aw_plain_left);
    print_bits("step_v", step.vel_w_i);
    manual_rotation = manual_rotation * SO3::exp(
        static_cast<Scalar>(t_ns - previous_data_t_ns) * Scalar(1e-9) *
        data.gyro);
    previous_data_t_ns = t_ns;
  }

  const auto& delta = measurement.getDeltaState();
  const Vec3 dp = delta.T_w_i.translation();
  const Vec3 dv = delta.vel_w_i;
  const Scalar dt = static_cast<Scalar>(measurement.get_dt_ns()) *
                    Scalar(1e-9);
  const SO3 r2(q2);
  const Vec3 rotated_dp = r2 * dp;
  const Vec3 rotated_dv = r2 * dv;
  const Vec3 g_dt = gravity * dt;
  const Vec3 g_half_dt = (Scalar(0.5) * gravity) * dt;
  const Vec3 g_half_dt_dt = g_half_dt * dt;

  // These scalar candidates deliberately keep all operands in f32.  The
  // first two match Eigen's ordinary expression-tree associations; the FMA
  // variants test whether a compiler contracts a gravity multiply/add lane.
  const Vec3 v_after_g = v2 + g_dt;
  const Vec3 v_source = v2 + g_dt + rotated_dv;
  const Vec3 v_group_right = v2 + (g_dt + rotated_dv);
  const Vec3 v_gravity_fma =
      Vec3(v2.x() + std::fma(gravity.x(), dt, 0.0f),
           v2.y() + std::fma(gravity.y(), dt, 0.0f),
           v2.z() + std::fma(gravity.z(), dt, 0.0f));
  const Vec3 v_fma_then_rot = v_gravity_fma + rotated_dv;
  const Vec3 v_rot_inside_fma =
      Vec3(std::fma(gravity.x(), dt, v2.x() + rotated_dv.x()),
           std::fma(gravity.y(), dt, v2.y() + rotated_dv.y()),
           std::fma(gravity.z(), dt, v2.z() + rotated_dv.z()));
  const Vec3 v_rot_then_gravity_fma =
      Vec3(std::fma(gravity.x(), dt, v2.x()) + rotated_dv.x(),
           std::fma(gravity.y(), dt, v2.y()) + rotated_dv.y(),
           std::fma(gravity.z(), dt, v2.z()) + rotated_dv.z());

  const Vec3 p_source = p2 + v2 * dt + g_half_dt_dt + rotated_dp;
  const Vec3 p_group_right = p2 + (v2 * dt + (g_half_dt_dt + rotated_dp));

  const basalt::PoseVelState<Scalar> state0(
      start_ns, Sophus::SE3<Scalar>(SO3(q2), p2), v2);
  basalt::PoseVelState<Scalar> state1;
  measurement.predictState(state0, gravity, state1);

  std::cout << std::setprecision(17);
  std::cout << "frame=" << (frame3 ? 3 : 2) << " dt=" << dt
            << " dt_ns=" << measurement.get_dt_ns()
            << " previous_data_t_ns=" << previous_data_t_ns << "\n";
  print_bits("q2_xyzw", q2.coeffs());
  print_bits("p2", p2);
  print_bits("v2", v2);
  print_bits("dp", dp);
  print_bits("dv", dv);
  print_bits("gravity", gravity);
  print_bits("g_dt", g_dt);
  print_bits("g_half_dt", g_half_dt);
  print_bits("g_half_dt_dt", g_half_dt_dt);
  print_bits("sophus_rotate_dp", rotated_dp);
  print_bits("sophus_rotate_dv", rotated_dv);
  print_bits("v_after_g", v_after_g);
  print_bits("v_source", v_source);
  print_bits("v_group_right", v_group_right);
  print_bits("v_gravity_fma", v_gravity_fma);
  print_bits("v_fma_then_rot", v_fma_then_rot);
  print_bits("v_rot_inside_fma", v_rot_inside_fma);
  print_bits("v_rot_then_gravity_fma", v_rot_then_gravity_fma);
  print_bits("p_source", p_source);
  print_bits("p_group_right", p_group_right);
  print_bits("direct_predict_p", state1.T_w_i.translation());
  print_bits("direct_predict_q", state1.T_w_i.so3().unit_quaternion().coeffs());
  print_bits("direct_predict_v", state1.vel_w_i);
  // Same-input residual position operand for the frame-3 -> frame-4 factor.
  // Keep this diagnostic next to the existing velocity boundary so the
  // Eigen matrix-vector reduction can be compared independently of delta-p.
  if (frame3) {
    const Vec3 p1(Scalar(0.007982099428772926),
                  Scalar(-0.0021397978998720646),
                  Scalar(-0.026876209303736687));
    const Vec3 pos_arg = p1 - p2 - v2 * dt - Scalar(0.5) * gravity * dt * dt;
    const auto r0_inv = r2.inverse().matrix();
    const Vec3 tmp_native = r0_inv * pos_arg;
    print_bits("residual_pos_arg", pos_arg);
    print_bits("residual_r0_inv", r0_inv);
    print_bits("residual_tmp_native", tmp_native);
    print_bits("residual_pos_native", tmp_native - dp);
    auto from_bits = [](std::uint32_t bits) {
      Scalar value;
      std::memcpy(&value, &bits, sizeof(value));
      return value;
    };
    auto r0_ordinary = r0_inv;
    r0_ordinary(1, 2) = from_bits(0xbd5ee27fU);
    r0_ordinary(2, 0) = from_bits(0xbf730653U);
    r0_ordinary(2, 1) = from_bits(0x3d96b5f6U);
    print_bits("residual_r0_ordinary", r0_ordinary);
    print_bits("residual_tmp_ordinary_eigen", r0_ordinary * pos_arg);
    print_bits("residual_tmp_ordinary_manual", Vec3(
        r0_ordinary(0, 0) * pos_arg.x() + r0_ordinary(0, 1) * pos_arg.y() +
            r0_ordinary(0, 2) * pos_arg.z(),
        r0_ordinary(1, 0) * pos_arg.x() + r0_ordinary(1, 1) * pos_arg.y() +
            r0_ordinary(1, 2) * pos_arg.z(),
        r0_ordinary(2, 0) * pos_arg.x() + r0_ordinary(2, 1) * pos_arg.y() +
            r0_ordinary(2, 2) * pos_arg.z()));
    const Scalar aa = r0_ordinary(2, 0) * pos_arg.x();
    const Scalar bb = r0_ordinary(2, 1) * pos_arg.y();
    const Scalar cc = r0_ordinary(2, 2) * pos_arg.z();
    print_bits("residual_row2_candidates", Vec3(
        (aa + bb) + cc,
        std::fma(r0_ordinary(2, 0), pos_arg.x(),
                 std::fma(r0_ordinary(2, 1), pos_arg.y(), cc)),
        std::fma(r0_ordinary(2, 0), pos_arg.x(), bb + cc)));
    print_bits("residual_row2_products", Vec3(aa, bb, cc));
  }
  return 0;
}
