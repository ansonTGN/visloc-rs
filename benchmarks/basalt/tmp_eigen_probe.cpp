#define main upstream_oracle_main
#include "upstream_sqrt_marginalization_oracle.cpp"
#undef main

bool trace_sum_packets = false;
int trace_sum_count = 0;

struct TraceSum {
  using result_type = float;
  float operator()(float a, float b) const { return a + b; }
  template <typename Packet>
  Packet packetOp(const Packet& a, const Packet& b) const {
    if (trace_sum_packets && trace_sum_count++ < 4) {
      float va[8] = {}, vb[8] = {};
      Eigen::internal::pstoreu(va, a);
      Eigen::internal::pstoreu(vb, b);
      std::cerr << "trace_packet_op";
      for (int i = 0; i < 8; ++i) std::cerr << " " << std::hex << bits(va[i]);
      std::cerr << " |";
      for (int i = 0; i < 8; ++i) std::cerr << " " << std::hex << bits(vb[i]);
      std::cerr << std::dec << "\n";
    }
    return Eigen::internal::padd(a, b);
  }
  template <typename Packet>
  float predux(const Packet& a) const {
    float values[8] = {};
    Eigen::internal::pstoreu(values, a);
    std::cerr << "trace_predux";
    for (int i = 0; i < 8; ++i) std::cerr << " " << std::hex << bits(values[i]);
    std::cerr << std::dec << "\n";
    return Eigen::internal::predux(a);
  }
};

namespace Eigen {
namespace internal {
template <>
struct functor_traits<TraceSum> {
  enum { Cost = 1, PacketAccess = 1 };
};
}  // namespace internal
}  // namespace Eigen

float manual_norm8(const float* data, int start, int len) {
  const int end = start + len;
  const std::uintptr_t byte_offset = (reinterpret_cast<std::uintptr_t>(data) + start * sizeof(float)) & 31U;
  const int peel = static_cast<int>(((32U - byte_offset) & 31U) / sizeof(float));
  const int aligned_start = start + (std::min)(peel, len);
  const int aligned_size = ((end - aligned_start) / 8) * 8;
  const int aligned_end = aligned_start + aligned_size;
  float result = 0.0f;
  if (aligned_size) {
    float packet[8];
    for (int lane = 0; lane < 8; ++lane) packet[lane] = data[aligned_start + lane] * data[aligned_start + lane];
    if (aligned_size > 8) {
      float packet1[8];
      for (int lane = 0; lane < 8; ++lane)
        packet1[lane] = data[aligned_start + 8 + lane] * data[aligned_start + 8 + lane];
      const int aligned_end2 = aligned_start + (aligned_size / 16) * 16;
      for (int index = aligned_start + 16; index < aligned_end2; index += 16) {
        for (int lane = 0; lane < 8; ++lane)
          packet[lane] += data[index + lane] * data[index + lane];
        for (int lane = 0; lane < 8; ++lane)
          packet1[lane] += data[index + 8 + lane] * data[index + 8 + lane];
      }
      for (int lane = 0; lane < 8; ++lane) packet[lane] += packet1[lane];
      if (aligned_end > aligned_end2)
        for (int lane = 0; lane < 8; ++lane)
          packet[lane] += data[aligned_end2 + lane] * data[aligned_end2 + lane];
    }
    const float pair0 = packet[0] + packet[4];
    const float pair1 = packet[1] + packet[5];
    const float pair2 = packet[2] + packet[6];
    const float pair3 = packet[3] + packet[7];
    const float even = pair0 + pair2;
    const float odd = pair1 + pair3;
    result = even + odd;
    for (int index = start; index < aligned_start; ++index) result += data[index] * data[index];
    for (int index = aligned_end; index < end; ++index) result += data[index] * data[index];
  } else {
    result = data[start] * data[start];
    for (int index = start + 1; index < end; ++index) result += data[index] * data[index];
  }
  return result;
}

float manual_norm4(const float* data, int start, int len) {
  const int end = start + len;
  const std::uintptr_t byte_offset = (reinterpret_cast<std::uintptr_t>(data) + start * sizeof(float)) & 15U;
  const int peel = static_cast<int>(((16U - byte_offset) & 15U) / sizeof(float));
  const int aligned_start = start + (std::min)(peel, len);
  const int aligned_size = ((end - aligned_start) / 4) * 4;
  const int aligned_end = aligned_start + aligned_size;
  float result = 0.0f;
  if (aligned_size) {
    float packet[4];
    for (int lane = 0; lane < 4; ++lane) packet[lane] = data[aligned_start + lane] * data[aligned_start + lane];
    for (int index = aligned_start + 4; index < aligned_end; index += 4)
      for (int lane = 0; lane < 4; ++lane) packet[lane] += data[index + lane] * data[index + lane];
    const float even = (packet[0] + packet[2]);
    const float odd = (packet[1] + packet[3]);
    result = even + odd;
    for (int index = start; index < aligned_start; ++index) result += data[index] * data[index];
    for (int index = aligned_end; index < end; ++index) result += data[index] * data[index];
  } else {
    result = data[start] * data[start];
    for (int index = start + 1; index < end; ++index) result += data[index] * data[index];
  }
  return result;
}

float manual_norm4_align32(const float* data, int start, int len) {
  const int end = start + len;
  const std::uintptr_t byte_offset = (reinterpret_cast<std::uintptr_t>(data) + start * sizeof(float)) & 31U;
  const int peel = static_cast<int>(((32U - byte_offset) & 31U) / sizeof(float));
  const int aligned_start = start + (std::min)(peel, len);
  const int aligned_size = ((end - aligned_start) / 4) * 4;
  const int aligned_end = aligned_start + aligned_size;
  float result = 0.0f;
  if (aligned_size) {
    float packet[4];
    for (int lane = 0; lane < 4; ++lane) packet[lane] = data[aligned_start + lane] * data[aligned_start + lane];
    for (int index = aligned_start + 4; index < aligned_end; index += 4)
      for (int lane = 0; lane < 4; ++lane) packet[lane] += data[index + lane] * data[index + lane];
    result = (packet[0] + packet[2]) + (packet[1] + packet[3]);
    for (int index = start; index < aligned_start; ++index) result += data[index] * data[index];
    for (int index = aligned_end; index < end; ++index) result += data[index] * data[index];
  } else {
    result = data[start] * data[start];
    for (int index = start + 1; index < end; ++index) result += data[index] * data[index];
  }
  return result;
}

float manual_scalar(const float* data, int start, int len) {
  float result = data[start] * data[start];
  for (int index = start + 1; index < start + len; ++index) result += data[index] * data[index];
  return result;
}

float eigen_packet8(const float* data, int start, int len) {
  using Packet = Eigen::internal::packet_traits<float>::type;
  const int end = start + len;
  const std::uintptr_t byte_offset = (reinterpret_cast<std::uintptr_t>(data) + start * sizeof(float)) & 31U;
  const int peel = static_cast<int>(((32U - byte_offset) & 31U) / sizeof(float));
  const int aligned_start = start + (std::min)(peel, len);
  const int aligned_size = ((end - aligned_start) / 8) * 8;
  const int aligned_end = aligned_start + aligned_size;
  float result = 0.0f;
  if (aligned_size) {
    Packet packet_result = Eigen::internal::pmul(
        Eigen::internal::ploadu<Packet>(data + aligned_start),
        Eigen::internal::ploadu<Packet>(data + aligned_start));
    if (aligned_size > 8) {
      Packet packet_result_1 = Eigen::internal::pmul(
          Eigen::internal::ploadu<Packet>(data + aligned_start + 8),
          Eigen::internal::ploadu<Packet>(data + aligned_start + 8));
      const int aligned_end2 = aligned_start + (aligned_size / 16) * 16;
      for (int index = aligned_start + 16; index < aligned_end2; index += 16) {
        packet_result = Eigen::internal::padd(
            packet_result,
            Eigen::internal::pmul(Eigen::internal::ploadu<Packet>(data + index),
                                  Eigen::internal::ploadu<Packet>(data + index)));
        packet_result_1 = Eigen::internal::padd(
            packet_result_1,
            Eigen::internal::pmul(Eigen::internal::ploadu<Packet>(data + index + 8),
                                  Eigen::internal::ploadu<Packet>(data + index + 8)));
      }
      packet_result = Eigen::internal::padd(packet_result, packet_result_1);
      if (aligned_end > aligned_end2)
        packet_result = Eigen::internal::padd(
            packet_result,
            Eigen::internal::pmul(Eigen::internal::ploadu<Packet>(data + aligned_end2),
                                  Eigen::internal::ploadu<Packet>(data + aligned_end2)));
    }
    result = Eigen::internal::predux(packet_result);
    for (int index = start; index < aligned_start; ++index) result += data[index] * data[index];
    for (int index = aligned_end; index < end; ++index) result += data[index] * data[index];
  } else {
    result = data[start] * data[start];
    for (int index = start + 1; index < end; ++index) result += data[index] * data[index];
  }
  return result;
}

int main(int argc, char** argv) {
  Matrix qj;
  Vector qr;
  std::set<int> keep;
  std::set<int> marg;
  if (!read_marg_binary(argv[1], qj, qr, keep, marg)) return 1;
  std::cerr << "base_mod32 " << (reinterpret_cast<std::uintptr_t>(qj.data()) & 31U) << "\n";
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(qj.cols());
  auto it = marg.begin();
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(marg.size()); ++k, ++it)
    indices[k] = *it;
  it = keep.begin();
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(keep.size()); ++k, ++it)
    indices[marg.size() + k] = *it;
  qj.applyOnTheRight(Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>>(indices));

  Eigen::Index total_rank = 0;
  Eigen::VectorXf temp;
  temp.resize(qj.cols() + 1);
  for (Eigen::Index k = 0; k < 60; ++k) {
    const Eigen::Index row_start = total_rank;
    const Eigen::Index remaining_rows = qj.rows() - row_start;
    const Eigen::Index remaining_cols = qj.cols() - k - 1;
    if (true) {
      auto tail_for_norm = qj.col(k).tail(remaining_rows - 1);
      auto norm_expr = tail_for_norm.unaryExpr(Eigen::internal::squared_norm_functor<float>());
      using NormExpr = decltype(norm_expr);
      using NormEval = Eigen::internal::redux_evaluator<NormExpr>;
      using NormTraits = Eigen::internal::redux_traits<Eigen::internal::squared_norm_functor<float>, NormEval>;
      std::cerr << "redux " << k << " packet " << NormTraits::PacketSize
                << " align " << Eigen::internal::unpacket_traits<typename NormTraits::PacketType>::alignment
                << " traversal " << NormTraits::Traversal << " flags " << NormEval::Flags
                << " first " << Eigen::internal::first_default_aligned(norm_expr) << "\n";
      if (k == 12 || k == 13 || k == 25) {
        NormEval eval(norm_expr);
        using Packet = typename NormTraits::PacketType;
        for (int pi = 0; pi < 2; ++pi) {
          Packet packet = eval.template packet<Eigen::Unaligned, Packet>(6 + pi * 8);
          float packet_values[8] = {};
          Eigen::internal::pstoreu(packet_values, packet);
          std::cerr << "eval_packet " << pi;
          for (int lane = 0; lane < 8; ++lane) std::cerr << " " << std::hex << bits(packet_values[lane]);
          std::cerr << std::dec << "\n";
        }
        std::cerr << "eval_coeff";
        for (int i = 6; i < 14; ++i) std::cerr << " " << std::hex << bits(eval.coeff(i));
        std::cerr << std::dec << "\n";
        trace_sum_packets = true;
        trace_sum_count = 0;
        std::cerr << "custom_sum " << k << " " << std::hex << bits(norm_expr.redux(TraceSum())) << std::dec << "\n";
        trace_sum_packets = false;
      }
      std::cerr << "norm " << k << " " << std::hex << bits(tail_for_norm.squaredNorm()) << " "
                << bits(norm_expr.sum()) << " "
                << bits(manual_norm8(qj.data(), static_cast<int>(row_start + 1 + k * qj.rows()),
                                     static_cast<int>(remaining_rows - 1)))
                << " "
                << bits(eigen_packet8(qj.data(), static_cast<int>(row_start + 1 + k * qj.rows()),
                                      static_cast<int>(remaining_rows - 1)))
                << " "
                << bits(manual_norm4(qj.data(), static_cast<int>(row_start + 1 + k * qj.rows()),
                                     static_cast<int>(remaining_rows - 1)))
                << " "
                << bits(manual_norm4_align32(qj.data(), static_cast<int>(row_start + 1 + k * qj.rows()),
                                              static_cast<int>(remaining_rows - 1)))
                << " "
                << bits(manual_scalar(qj.data(), static_cast<int>(row_start + 1 + k * qj.rows()),
                                      static_cast<int>(remaining_rows - 1)))
                << std::dec << "\n";
    }
    float beta = 0.0f;
    float h = 0.0f;
    qj.col(k).tail(remaining_rows).makeHouseholderInPlace(h, beta);
    qj.coeffRef(total_rank, k) = beta;
    auto derived = qj.bottomRightCorner(remaining_rows, remaining_cols);
    auto bottom = derived.block(1, 0, remaining_rows - 1, remaining_cols);
    auto essential = qj.col(k).tail(remaining_rows - 1);
    auto trans = bottom.transpose();
    std::cerr << "k " << k << " rows " << remaining_rows << " cols " << remaining_cols
              << " bottom rows " << bottom.rows() << " cols " << bottom.cols()
              << " bottom inner " << bottom.innerStride() << " outer " << bottom.outerStride()
              << " trans inner " << trans.innerStride() << " outer " << trans.outerStride()
              << " e ptr delta " << (essential.data() - qj.data())
              << " b ptr delta " << (bottom.data() - qj.data()) << "\n";
    Eigen::Map<Eigen::RowVectorXf> workspace(temp.data() + k + 1, remaining_cols);
    workspace.noalias() = essential.adjoint() * bottom;
    std::cerr << "temp";
    for (int i = 0; i < (std::min)(8, static_cast<int>(remaining_cols)); ++i)
      std::cerr << " " << std::hex << bits(temp[k + 1 + i]);
    std::cerr << std::dec << "\n";
    derived.applyHouseholderOnTheLeft(essential, h, temp.data() + k + 1);
    essential.setZero();
    ++total_rank;
    std::cerr << "hash " << k << " " << std::hex << matrix_bits_hash(qj) << std::dec << "\n";
  }
  return 0;
}
