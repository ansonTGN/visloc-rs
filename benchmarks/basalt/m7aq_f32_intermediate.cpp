#include <basalt/imu/preintegration.h>
#include <iomanip>
#include <iostream>
#include <cstdint>
#include <cstring>
static uint32_t bits(float x) { uint32_t u; std::memcpy(&u,&x,4); return u; }
int main() {
  using S=float; using V=Eigen::Matrix<S,3,1>; using SO3=Sophus::SO3<S>;
  V gyro(-0.117891803383827f,-0.0232203491032124f,-0.0431733503937721f);
  V accel(7.73393440246582f,-0.577644169330597f,-3.00091099739075f);
  S dt = static_cast<S>(4999936LL) * S(1e-9);
  SO3 half = SO3::exp(S(0.5) * dt * gyro);
  SO3 full = SO3::exp(dt * gyro);
  SO3::QuaternionType qh = half.unit_quaternion();
  auto R = half.matrix(); V aw = R * accel;
  std::cout << std::setprecision(15);
  std::cout << "dt " << dt << "\nqh " << qh.coeffs().transpose() << "\nR\n" << R << "\naw " << aw.transpose() << "\nfull " << full.unit_quaternion().coeffs().transpose() << "\n";
  std::cout << std::hex << "bits dt " << bits(dt) << " q " << bits(qh.x()) << " " << bits(qh.y()) << " " << bits(qh.z()) << " " << bits(qh.w()) << " aw " << bits(aw.x()) << " " << bits(aw.y()) << " " << bits(aw.z()) << " full " << bits(full.unit_quaternion().x()) << " " << bits(full.unit_quaternion().y()) << " " << bits(full.unit_quaternion().z()) << " " << bits(full.unit_quaternion().w()) << "\n";
  V g2(-0.115099273622036f,-0.0399755090475082f,-0.0508527979254723f);
  V a2(7.84017324447632f,-0.569471955299377f,-2.91918873786926f);
  SO3 R2 = full; V v2=aw*dt; V p2=aw*(S(0.5)*dt)*dt;
  S dt2 = static_cast<S>(5000192LL) * S(1e-9);
  SO3 h2 = R2 * SO3::exp(S(0.5)*dt2*g2); V aw2=h2.matrix()*a2;
  p2 += v2*dt2 + aw2*(S(0.5)*dt2)*dt2; v2 += aw2*dt2; R2=R2*SO3::exp(dt2*g2);
  auto e2=SO3::exp(dt2*g2).unit_quaternion(); auto hh=h2.unit_quaternion();
  std::cout << std::hex << "e2 " << bits(e2.x()) << " " << bits(e2.y()) << " " << bits(e2.z()) << " " << bits(e2.w()) << " h2 " << bits(hh.x()) << " " << bits(hh.y()) << " " << bits(hh.z()) << " " << bits(hh.w()) << "\n";
  std::cout << std::hex << "second p " << bits(p2.x()) << " " << bits(p2.y()) << " " << bits(p2.z()) << " v " << bits(v2.x()) << " " << bits(v2.y()) << " " << bits(v2.z()) << " q " << bits(R2.unit_quaternion().x()) << " " << bits(R2.unit_quaternion().y()) << " " << bits(R2.unit_quaternion().z()) << " " << bits(R2.unit_quaternion().w()) << " aw " << bits(aw2.x()) << " " << bits(aw2.y()) << " " << bits(aw2.z()) << "\n";
  const V gs[10] = {
    {-0.117891803383827f,-0.0232203491032124f,-0.0431733503937721f},
    {-0.115099273622036f,-0.0399755090475082f,-0.0508527979254723f},
    {-0.108117960393429f,-0.0637119859457016f,-0.0613247714936733f},
    {-0.105325430631638f,-0.0853540748357773f,-0.0704004839062691f},
    {-0.0962497219443321f,-0.102109231054783f,-0.0759855359792709f},
    {-0.0871740058064461f,-0.11397746950388f,-0.0780799314379692f},
    {-0.0760039016604424f,-0.119562521576881f,-0.0801743268966675f},
    {-0.0690225809812546f,-0.127940103411674f,-0.0752874091267586f},
    {-0.0634375289082527f,-0.14190274477005f,-0.0669098272919655f},
    {-0.0606450065970421f,-0.150978446006775f,-0.0585322454571724f}};
  const V as[10] = {
    {7.73393440246582f,-0.577644169330597f,-3.00091099739075f},
    {7.84017324447632f,-0.569471955299377f,-2.91918873786926f},
    {8.03630542755127f,-0.544955372810364f,-2.84563899040222f},
    {8.13437271118164f,-0.520438730716705f,-2.78026127815247f},
    {8.25695514678955f,-0.487749874591827f,-2.72305583953857f},
    {8.35502243041992f,-0.536783158779144f,-2.6903669834137f},
    {8.297816276550293f,-0.471405506134033f,-2.62498927116394f},
    {8.216094017028809f,-0.504094302654266f,-2.67402267456055f},
    {8.24878311157227f,-0.528610944747925f,-2.73940014839172f},
    {8.11802768707275f,-0.561299800872803f,-2.84563899040222f}};
  const int64_t ts[10] = {4999936,5000192,4999936,4999936,4999936,5000192,4999936,4999936,4999936,5000192};
  SO3 RR; V vv=V::Zero(), pp=V::Zero();
  for(int i=0;i<10;i++){ S d=static_cast<S>(ts[i])*S(1e-9); SO3 hh=RR*SO3::exp(S(0.5)*d*gs[i]); V aa=hh.matrix()*as[i]; V oldp=pp, vdt=vv*d, ah=aa*S(0.5), ahdt=ah*d, ahdtdt=ahdt*d, term=0.5*aa*d*d; pp += vv*d + 0.5*aa*d*d; if(i==3) std::cout << std::hex << "ZBOUND prev=" << bits(oldp.z()) << " vdt=" << bits(vdt.z()) << " ah=" << bits(ah.z()) << " ahdt=" << bits(ahdt.z()) << " ahdtdt=" << bits(ahdtdt.z()) << " term=" << bits(term.z()) << " sum=" << bits(oldp.z()+vdt.z()) << " sum2=" << bits((oldp.z()+vdt.z())+ahdtdt.z()) << " out=" << bits(pp.z()) << "\n"; vv += aa*d; RR=RR*SO3::exp(d*gs[i]); std::cout << std::hex << "step " << i << " p " << bits(pp.x()) << " " << bits(pp.y()) << " " << bits(pp.z()) << " v " << bits(vv.x()) << " " << bits(vv.y()) << " " << bits(vv.z()) << "\n"; }
  std::cout << std::hex << "final p " << bits(pp.x()) << " " << bits(pp.y()) << " " << bits(pp.z()) << " v " << bits(vv.x()) << " " << bits(vv.y()) << " " << bits(vv.z()) << " q " << bits(RR.unit_quaternion().x()) << " " << bits(RR.unit_quaternion().y()) << " " << bits(RR.unit_quaternion().z()) << " " << bits(RR.unit_quaternion().w()) << "\n";
}
