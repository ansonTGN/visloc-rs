#include <cmath>
#include <Eigen/Core>
#include <immintrin.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

float from_bits(std::uint32_t x) { float y; std::memcpy(&y, &x, 4); return y; }
std::uint32_t bits(float x) { std::uint32_t y; std::memcpy(&y, &x, 4); return y; }
std::vector<std::uint32_t> words(const std::string& s) {
  std::vector<std::uint32_t> out; std::stringstream ss(s); std::string t;
  while (std::getline(ss, t, ',')) if (!t.empty()) out.push_back(std::stoul(t, nullptr, 16));
  return out;
}
std::string field(const std::string& line, const char* name) {
  std::string p = std::string(name) + "="; auto b = line.find(p);
  if (b == std::string::npos) throw std::runtime_error("missing field");
  b += p.size(); auto e = line.find(' ', b); return line.substr(b, e == std::string::npos ? e : e-b);
}
struct Record { std::vector<std::uint32_t> a, g, t2, t3; float av, gv; };
float product(float x, float y) { return x * y; }
float chain(const float* x, const float* y, const int* o) {
  float v = product(x[o[0]], y[o[0]]);
  v = std::fma(x[o[1]], y[o[1]], v);
  return std::fma(x[o[2]], y[o[2]], v);
}
float pair_add_fma(const float* x, const float* y, const int* o) {
  float v = product(x[o[0]], y[o[0]]) + product(x[o[1]], y[o[1]]);
  return std::fma(x[o[2]], y[o[2]], v);
}
float pair_add_add(const float* x, const float* y, const int* o) {
  float v = product(x[o[0]], y[o[0]]) + product(x[o[1]], y[o[1]]);
  return v + product(x[o[2]], y[o[2]]);
}
float fma_pair_fma(const float* x, const float* y, const int* o) {
  float v = std::fma(x[o[0]], y[o[0]], product(x[o[1]], y[o[1]]));
  return std::fma(x[o[2]], y[o[2]], v);
}
float fma_pair_add(const float* x, const float* y, const int* o) {
  float v = std::fma(x[o[0]], y[o[0]], product(x[o[1]], y[o[1]]));
  return v + product(x[o[2]], y[o[2]]);
}
float packet_tail(const float* x, const float* y) {
  const __m256 b01 = _mm256_set_ps(y[1], y[1], y[1], y[1], y[0], y[0], y[0], y[0]);
  const __m256 ax01 = _mm256_set_ps(x[1], x[1], x[1], x[1], x[0], x[0], x[0], x[0]);
  const __m256 c = _mm256_fmadd_ps(ax01, b01, _mm256_setzero_ps());
  const __m128 half = _mm_add_ps(_mm256_castps256_ps128(c), _mm256_extractf128_ps(c, 1));
  const __m128 out = _mm_fmadd_ps(_mm_set1_ps(x[2]), _mm_loadu_ps(y+2), half);
  return _mm_cvtss_f32(out);
}
float packet_scalar(const float* x, const float* y) {
  const float p0 = std::fma(x[0], y[0], 0.0f);
  const float p1 = std::fma(x[1], y[1], 0.0f);
  const float pair = p0 + p1;
  return std::fma(x[2], y[2], pair);
}
using Fn = float (*)(const float*, const float*, const int*);
using MatA = Eigen::Matrix<float, 9, 3>;
using MatC = Eigen::Matrix<float, 3, 1>;
using MatE = Eigen::Matrix<float, 9, 9>;
__attribute__((noinline)) MatE eigen_weighted(const MatA& a, const MatC& c) {
  return a * c.asDiagonal() * a.transpose();
}
__attribute__((noinline)) MatA eigen_scaled(const MatA& a, const MatC& c) {
  return a * c.asDiagonal();
}
int main(int argc, char** argv) {
  std::ifstream in(argc > 1 ? argv[1] : "target/m7hd_native_intermediate.jsonl");
  std::vector<Record> rec; std::string line; Record r;
  while (std::getline(in, line)) {
    if (line.rfind("M7HD_NATIVE_STEP", 0) == 0) {
      r.a = words(field(line, "a")); r.g = words(field(line, "g"));
    } else if (line.rfind("M7HD_NATIVE_COV", 0) == 0) {
      r.t2 = words(field(line, "term2")); r.t3 = words(field(line, "term3"));
      r.av = from_bits(std::stoul(field(line, "accel_cov"), nullptr, 16));
      r.gv = from_bits(std::stoul(field(line, "gyro_cov"), nullptr, 16)); rec.push_back(r);
    }
  }
  int eigen_miss_a=0, eigen_miss_g=0, scale_miss=0;
  for (const auto& p : rec) {
    MatA a, g; MatC av, gv;
    for (int i=0;i<27;++i) { a.data()[i]=from_bits(p.a[i]); g.data()[i]=from_bits(p.g[i]); }
    for (int i=0;i<3;++i) { av[i]=p.av; gv[i]=p.gv; }
    const MatA sa=eigen_scaled(a,av), sg=eigen_scaled(g,gv);
    const MatE ea=eigen_weighted(a,av), eg=eigen_weighted(g,gv);
    for (int k=0;k<3;++k) {
      const float want=from_bits(p.a[8+9*k])*p.av;
      if (bits(sa.data()[8+9*k])!=bits(want)) { ++scale_miss; std::cout << "scale A packet=" << (&p-&rec[0]+1) << " k=" << k << " got=" << std::hex << bits(sa.data()[8+9*k]) << " want=" << bits(want) << std::dec << '\n'; }
    }
    for (int i=0;i<81;++i) { if (bits(ea.data()[i])!=p.t2[i]) ++eigen_miss_a; if (bits(eg.data()[i])!=p.t3[i]) ++eigen_miss_g; }
  }
  std::cout << "Eigen all A=" << eigen_miss_a << "/" << rec.size()*81
            << " G=" << eigen_miss_g << "/" << rec.size()*81
            << " scaleA=" << scale_miss << "/" << rec.size()*3 << '\n';
  const int orders[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
  const char* onames[6] = {"012","021","102","120","201","210"};
  const Fn fns[5] = {chain,pair_add_fma,pair_add_add,fma_pair_fma,fma_pair_add};
  const char* fnames[5] = {"chain","pairaddfma","pairaddadd","fmapairfma","fmapairadd"};
  int pmiss=0;
  for (std::size_t p=0;p<rec.size();++p) for(int c=0;c<9;++c) {
    float x[3],y[3]; for(int k=0;k<3;++k){x[k]=from_bits(rec[p].a[8+9*k])*rec[p].av;y[k]=from_bits(rec[p].a[c+9*k]);}
    if(c<8) { float v=packet_tail(x,y); if(bits(v)!=rec[p].t2[8+9*c]){++pmiss;if(pmiss<10)std::cout<<"packet A p="<<p+1<<" c="<<c<<" got="<<std::hex<<bits(v)<<" want="<<rec[p].t2[8+9*c]<<std::dec<<'\n';} }
  }
  std::cout << "packet_tail A=" << pmiss << "/" << rec.size()*8 << '\n';
  int psmiss=0, c8miss=0;
  for (std::size_t p=0;p<rec.size();++p) for(int c=0;c<9;++c) {
    float x[3],y[3]; for(int k=0;k<3;++k){x[k]=from_bits(rec[p].a[8+9*k])*rec[p].av;y[k]=from_bits(rec[p].a[c+9*k]);}
    float v = c<8 ? packet_scalar(x,y) : std::fma(x[2],y[2],std::fma(x[1],y[1],std::fma(x[0],y[0],0.0f)));
    if(bits(v)!=rec[p].t2[8+9*c]) { ++psmiss; if(c==8)++c8miss; }
  }
  std::cout << "packet_scalar A=" << psmiss << "/" << rec.size()*9 << " c8=" << c8miss << '\n';
  {
    const auto& q=rec[2]; float x[3],y[3]; for(int k=0;k<3;++k){x[k]=from_bits(q.a[8+9*k])*q.av;y[k]=from_bits(q.a[0+9*k]);}
    std::cout<<"witness p3c0 x="<<std::hex<<bits(x[0])<<","<<bits(x[1])<<","<<bits(x[2])<<" y="<<bits(y[0])<<","<<bits(y[1])<<","<<bits(y[2])<<" pair="<<bits(pair_add_fma(x,y,orders[0]))<<" packet="<<bits(packet_tail(x,y))<<" scalar="<<bits(packet_scalar(x,y))<<" want="<<q.t2[8]<<std::dec<<'\n';
  }
  for (int fi=0; fi<5; ++fi) for (int oi=0; oi<6; ++oi) {
    int miss=0; int firstp=-1, firstc=-1; std::uint32_t got=0, want=0;
    for (std::size_t p=0; p<rec.size(); ++p) for (int c=0; c<9; ++c) {
      float x[3], y[3];
      for (int k=0;k<3;++k) { x[k]=from_bits(rec[p].a[8+9*k]) * rec[p].av; y[k]=from_bits(rec[p].a[c+9*k]); }
      float v=fns[fi](x,y,orders[oi]);
      if (bits(v)!=rec[p].t2[8+9*c]) { ++miss; if(firstp<0){firstp=p;firstc=c;got=bits(v);want=rec[p].t2[8+9*c];} }
    }
    std::cout << "A " << fnames[fi] << onames[oi] << " mismatches=" << miss << "/" << rec.size()*9;
    if(firstp>=0) std::cout << " first=" << firstp+1 << ":" << firstc << " actual=" << std::hex << got << " expected=" << want << std::dec;
    std::cout << '\n';
  }
  for (int fi=0; fi<5; ++fi) for (int oi=0; oi<6; ++oi) {
    int miss=0; int firstp=-1, firstc=-1; std::uint32_t got=0, want=0;
    for (std::size_t p=0; p<rec.size(); ++p) for (int c=0; c<9; ++c) {
      float x[3], y[3];
      for (int k=0;k<3;++k) { x[k]=from_bits(rec[p].g[8+9*k]) * rec[p].gv; y[k]=from_bits(rec[p].g[c+9*k]); }
      float v=fns[fi](x,y,orders[oi]);
      if (bits(v)!=rec[p].t3[8+9*c]) { ++miss; if(firstp<0){firstp=p;firstc=c;got=bits(v);want=rec[p].t3[8+9*c];} }
    }
    std::cout << "G " << fnames[fi] << onames[oi] << " mismatches=" << miss << "/" << rec.size()*9;
    if(firstp>=0) std::cout << " first=" << firstp+1 << ":" << firstc << " actual=" << std::hex << got << " expected=" << want << std::dec;
    std::cout << '\n';
  }
}
