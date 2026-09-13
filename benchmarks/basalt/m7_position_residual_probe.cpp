// Diagnostic-only probe for the f32 Eigen expression tree used by
// IntegratedImuMeasurement::residual.  This file is not production code.
#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

using V = Eigen::Matrix<float, 3, 1>;

static std::string bits(float x) {
  std::uint32_t u = 0;
  std::memcpy(&u, &x, sizeof(u));
  char b[9];
  std::snprintf(b, sizeof(b), "%08x", u);
  return b;
}

static void dump(const char* label, const V& v) {
  std::cout << label << "=" << bits(v.x()) << "," << bits(v.y()) << ","
            << bits(v.z()) << "\n";
}

int main() {
  const V p0 = (V() << 0x1.1d7f40p-8f, -0x1.2851e8p-10f,
                -0x1.0b0aecp-6f)
                   .finished();
  const V p1 = (V() << 0x1.0574b4p-7f, -0x1.1877b74p-9f,
                -0x1.b0afep-6f)
                   .finished();
  const V v0 = (V() << 0x1.0a088p-4f, -0x1.33224p-6f,
                -0x1.94a818p-3f)
                   .finished();
  const V g(0.0f, 0.0f, -9.8100004196166992f);
  const float dt = 0x1.9999c8p-5f;

  // The actual decimal values above are replaced below by exact f32 bit
  // loads, avoiding any source-literal conversion ambiguity.
  auto f = [](std::uint32_t u) {
    float x;
    std::memcpy(&x, &u, sizeof(x));
    return x;
  };
  const V P0(f(0x3b8ebfa0), f(0xba9428f4), f(0xbc857642));
  const V P1(f(0x3c02c75a), f(0xbb0c3bda), f(0xbcdc2b7f));
  const V V0(f(0x3d850448), f(0xbc998d12), f(0xbe4a560c));
  const V G(f(0), f(0), f(0xc11cf5c3));
  const float DT = f(0x3d4cccef);
  dump("inputs_p0", P0);
  dump("inputs_p1", P1);
  dump("inputs_v0", V0);
  dump("inputs_g", G);
  std::cout << "dt=" << bits(DT) << "\n";

  // Header expression, source spelling, in one Eigen assignment.
  const V source = P1 - P0 - V0 * DT - 0.5f * G * DT * DT;
  dump("source", source);

  const V source_group = P1 - P0 - V0 * DT - (0.5f * G * DT * DT);
  dump("source_group", source_group);
  const V source_left = ((P1 - P0) - V0 * DT) - 0.5f * G * DT * DT;
  dump("source_left", source_left);
  const V source_right = (P1 - P0) - (V0 * DT + 0.5f * G * DT * DT);
  dump("source_right", source_right);
  const V source_pv = P1 - (P0 + V0 * DT) - 0.5f * G * DT * DT;
  dump("source_pv", source_pv);

  const V base = P1 - P0;
  const V vdt = V0 * DT;
  const V half_g = 0.5f * G;
  const V half_g_dt = half_g * DT;
  const V half_g_dt_dt = half_g_dt * DT;
  dump("base", base);
  dump("vdt", vdt);
  dump("half_g", half_g);
  dump("half_g_dt", half_g_dt);
  dump("half_g_dt_dt", half_g_dt_dt);
  dump("base_minus_vdt", base - vdt);
  dump("base_minus_vdt_minus_g", base - vdt - half_g_dt_dt);

  // Explicit scalar contraction candidates.
  V fma1, fma2, fma3, fma4;
  for (int i = 0; i < 3; ++i) {
    fma1[i] = std::fma(-V0[i], DT, P1[i] - P0[i]);
    fma2[i] = std::fma(-G[i], 0.5f * DT * DT, fma1[i]);
    fma3[i] = std::fma(-G[i], (0.5f * DT) * DT, fma1[i]);
    fma4[i] = std::fma(-G[i], DT, std::fma(-V0[i], DT, P1[i] - P0[i]));
  }
  dump("fma_half_dt2", fma2);
  dump("fma_half_dt_then_dt", fma3);
  dump("fma_nested_g_dt", fma4);
}
