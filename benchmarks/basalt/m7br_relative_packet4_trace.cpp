// M7br: safe scalar replay of the inlined Eigen Packet4f block at
// LinearizationAbsQR<float,6>::linearizeProblem + 0x708 .. +0x7c4
// (file addresses 0x2e6208 .. 0x2e62c4 in the pinned oracle).
//
// This is deliberately a diagnostic, not production code.  The arrays model
// four f32 lanes; std::fma is the scalar spelling of a packed FMA lane.  No
// target-specific intrinsic or raw pointer is used.  Expected bit patterns
// below are the captured native lanes in m7br_relative_packet4_native_trace.txt.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using P = std::array<float, 4>;

static float fma(float a, float b, float c) { return std::fma(a, b, c); }

static P perm(const P& p, unsigned imm) {
  return {p[(imm >> 0) & 3], p[(imm >> 2) & 3], p[(imm >> 4) & 3],
          p[(imm >> 6) & 3]};
}

static P mul(const P& a, const P& b) {
  return {a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]};
}

static P add(const P& a, const P& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

static P sub_fma(const P& a, const P& b, const P& c) {
  return {fma(a[0], b[0], -c[0]), fma(a[1], b[1], -c[1]),
          fma(a[2], b[2], -c[2]), fma(a[3], b[3], -c[3])};
}

static P add_fma(const P& a, const P& b, const P& c) {
  return {fma(a[0], b[0], c[0]), fma(a[1], b[1], c[1]),
          fma(a[2], b[2], c[2]), fma(a[3], b[3], c[3])};
}

static P neg_mul_add(const P& a, const P& b, const P& c) {
  return {fma(-a[0], b[0], c[0]), fma(-a[1], b[1], c[1]),
          fma(-a[2], b[2], c[2]), fma(-a[3], b[3], c[3])};
}

static P blend_lane3(const P& base, const P& lane) {
  P out = base;
  out[3] = lane[3];
  return out;
}

static uint32_t bits(float x) {
  uint32_t u = 0;
  std::memcpy(&u, &x, sizeof(u));
  return u;
}

static float from_bits(uint32_t u) {
  float x = 0.0f;
  std::memcpy(&x, &u, sizeof(x));
  return x;
}

static void show(const char* label, const P& p) {
  std::cout << label << '=' << std::hex << std::setfill('0');
  for (float x : p) std::cout << std::setw(8) << bits(x) << ',';
  std::cout << std::dec << '\n';
}

static void verify(const char* label, const P& p,
                   const std::array<uint32_t, 4>& expected) {
  show(label, p);
  for (size_t lane = 0; lane != 4; ++lane) {
    if (bits(p[lane]) != expected[lane]) {
      std::cerr << "lane mismatch at " << label << "[" << lane << "]: got "
                << std::hex << bits(p[lane]) << ", expected " << expected[lane]
                << std::dec << '\n';
      std::exit(EXIT_FAILURE);
    }
  }
}

int main() {
  // Runtime operands captured at the target frame-0 -> frame-1 pair.  Eigen
  // stores Quaternion coefficients as [x,y,z,w].
  const P a = {from_bits(0x3d5cdea6), from_bits(0x3f4d341f),
               from_bits(0x3b2aa78b), from_bits(0x3f186f67)};
  const P b = {from_bits(0xbd582e43), from_bits(0xbf4d686f),
               from_bits(0x00000000), from_bits(0x3f182ffd)};
  verify("input_inverse_target", a,
         {0x3d5cdea6, 0x3f4d341f, 0x3b2aa78b, 0x3f186f67});
  verify("input_host", b,
         {0xbd582e43, 0xbf4d686f, 0x00000000, 0x3f182ffd});

  const P p3f = perm(b, 0x3f);
  const P p52 = perm(b, 0x52);
  const P p89 = perm(b, 0x89);
  const P p49 = perm(a, 0x49);
  const P pff = perm(a, 0xff);
  const P p92 = perm(a, 0x92);
  verify("perm_b_3f", p3f,
         {0x3f182ffd, 0x3f182ffd, 0x3f182ffd, 0xbd582e43});
  verify("perm_b_52", p52,
         {0x00000000, 0xbd582e43, 0xbf4d686f, 0xbf4d686f});
  verify("perm_b_89", p89,
         {0xbf4d686f, 0x00000000, 0xbd582e43, 0x00000000});
  verify("perm_a_49", p49,
         {0x3f4d341f, 0x3b2aa78b, 0x3d5cdea6, 0x3f4d341f});
  verify("perm_a_ff", pff,
         {0x3f186f67, 0x3f186f67, 0x3f186f67, 0x3f186f67});
  verify("perm_a_92", p92,
         {0x3b2aa78b, 0x3d5cdea6, 0x3f4d341f, 0x3b2aa78b});
  P x0 = perm(a, 0x24);
  verify("post_624a_x0", x0,
         {0x3d5cdea6, 0x3f4d341f, 0x3b2aa78b, 0x3d5cdea6});
  x0 = mul(x0, p3f);
  verify("post_6250_x0_mul", x0,
         {0x3d034d9a, 0x3ef3fad4, 0x3acae6f0, 0xbb3a83c6});
  P x12 = pff;
  verify("post_6255_x12_copy", x12,
         {0x3f186f67, 0x3f186f67, 0x3f186f67, 0x3f186f67});
  x12 = sub_fma(x12, b, x0);  // vfmsub132ps
  verify("post_6259_x12", x12,
         {0xbd820392, 0xbf744ccf, 0xbacae6f0, 0x3eb6b278});
  P x2 = add_fma(pff, b, x0);  // vfmadd132ps
  verify("post_625e_x2", x2,
         {0x3a2503cc, 0xbaa3f5aa, 0x3acae6f0, 0x3eb3c869});
  x2 = blend_lane3(x2, x12);  // vblendps $0x8
  verify("post_6267_x2_blend", x2,
         {0x3a2503cc, 0xbaa3f5aa, 0x3acae6f0, 0x3eb6b278});
  x0 = p49;  // vmovaps %xmm1,%xmm0 at 0x6263
  verify("post_6263_x0_copy", x0,
         {0x3f4d341f, 0x3b2aa78b, 0x3d5cdea6, 0x3f4d341f});
  x0 = neg_mul_add(p49, p52, x2);  // vfnmadd132ps: x2 - p49*p52
  verify("post_626d_x0", x0,
         {0x3a2503cc, 0xba91f223, 0x3d378f9d, 0x3f7fffaf});
  x2 = add_fma(p52, p49, x2);  // vfmadd231ps
  verify("post_6272_x2", x2,
         {0x3a2503cc, 0xbab5f931, 0xbd2ae12e, 0xbe929a6e});
  x2 = blend_lane3(x2, x0);
  verify("post_6277_x2_blend", x2,
         {0x3a2503cc, 0xbab5f931, 0xbd2ae12e, 0x3f7fffaf});
  x2 = neg_mul_add(p89, p92, x2);  // vfnmadd231ps
  verify("post_627d_raw", x2,
         {0x3b322ebb, 0xbab5f931, 0x3a19f84f, 0x3f7fffaf});

  const P sq = mul(x2, x2);
  verify("post_6282_sq", sq,
         {0x36f80a09, 0x36015a52, 0x34b9357f, 0x3f7fff5e});
  verify("post_6286_x2_copy", x2,
         {0x3b322ebb, 0xbab5f931, 0x3a19f84f, 0x3f7fffaf});
  const P hi = {sq[2], sq[3], sq[2], sq[3]};  // vmovhlps
  verify("post_628e_x1", hi,
         {0x34b9357f, 0x3f7fff5e, 0x34b9357f, 0x3f7fff5e});
  const P s1 = add(sq, hi);                  // vaddps
  verify("post_6292_pair_sum", s1,
         {0x3701ceb0, 0x3f7fff7e, 0x3539357f, 0x3fffff5e});
  const P dup = {s1[1], s1[1], s1[3], s1[3]};  // vmovshdup
  verify("post_6296_x1", dup,
         {0x3f7fff7e, 0x3f7fff7e, 0x3fffff5e, 0x3fffff5e});
  const P s2 = add(s1, dup);                   // vaddps
  verify("post_629a_norm2", s2,
         {0x3f800000, 0x3fffff7e, 0x3fffff64, 0x407fff5e});
  // xmm7's scalar add at 0x629e is zero for this call.  Keep the packet
  // materialized so the scalar reduction and vinsertps are explicit too.
  const P s3 = s2;
  verify("post_629e_addss", s3,
         {0x3f800000, 0x3fffff7e, 0x3fffff64, 0x407fff5e});
  const P inserted = {s3[0], 0.0f, 0.0f, 0.0f};
  verify("post_62a2_insertps", inserted,
         {0x3f800000, 0x00000000, 0x00000000, 0x00000000});
  const float norm = std::sqrt(inserted[0]);
  const P sqrt_packet = {norm, inserted[1], inserted[2], inserted[3]};
  verify("post_62a8_sqrt", sqrt_packet,
         {0x3f800000, 0x00000000, 0x00000000, 0x00000000});
  const P n = perm(sqrt_packet, 0x00);
  verify("post_62ba_divisor", n,
         {0x3f800000, 0x3f800000, 0x3f800000, 0x3f800000});
  verify("post_62c0_normalized",
         {x2[0] / n[0], x2[1] / n[1], x2[2] / n[2], x2[3] / n[3]},
         {0x3b322ebb, 0xbab5f931, 0x3a19f84f, 0x3f7fffaf});
}
