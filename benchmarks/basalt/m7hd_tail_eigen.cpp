#include <Eigen/Core>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

static float F(std::uint32_t x) { float y; std::memcpy(&y,&x,4); return y; }
static std::uint32_t B(float x) { std::uint32_t y; std::memcpy(&y,&x,4); return y; }
using MatA = Eigen::Matrix<float,9,3>;
using MatC = Eigen::Matrix<float,3,1>;
using MatE = Eigen::Matrix<float,9,9>;
__attribute__((noinline)) MatE compute(const MatA& A, const MatC& C) {
  return A * C.asDiagonal() * A.transpose();
}
static std::vector<std::uint32_t> field(const std::string& l,const char*n) {
  std::regex r(std::string("\\b")+n+               +"=([^ ]+)"); std::smatch m; if(!std::regex_search(l,m,r)) throw 1;
  std::vector<std::uint32_t> v; std::stringstream s(m[1].str()),t; std::string x;
  while(std::getline(s,x,',')) v.push_back(std::stoul(x,nullptr,16)); return v;
}
int main(int ac,char**av) {
  std::ifstream in(ac>1?av[1]:"target/m7hd_native_intermediate.jsonl"); std::string l; std::vector<std::uint32_t>a,t,c;
  while(std::getline(in,l)) { if(l.rfind("M7HD_NATIVE_STEP",0)==0) a=field(l,"a"); if(l.rfind("M7HD_NATIVE_COV",0)==0){t=field(l,"term2");c=field(l,"accel_cov");break;} }
  std::cerr << "sizes " << a.size() << " " << t.size() << " " << c.size()
            << " a0 " << std::hex << a[0] << " a8 " << a[8] << " a17 " << a[17]
            << " a26 " << a[26] << " c0 " << c[0] << std::dec << "\n";
  MatA A; MatC C; MatE E;
  for(int i=0;i<27;i++) A.data()[i]=F(a[i]); for(int i=0;i<3;i++)C[i]=F(c[i]); E=compute(A,C);
  for(int col=0;col<9;col++) std::cout<<col<<" direct="<<std::hex<<B(E(8,col))<<" native="<<t[8+9*col]<<std::dec<<"\n";
}
