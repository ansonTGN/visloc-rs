#include <Eigen/Core>
#include <Eigen/QR>
#include <nlohmann/json.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>
#include <fstream>
#include <array>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

using json = nlohmann::json;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;

static const json &feature(const json &raw, std::uint64_t frame, int cam) {
  for (const auto &item : raw.at("features"))
    if (item.at("time_cam_id").at("frame_id") == frame && item.at("time_cam_id").at("cam_id") == cam)
      return item;
  throw std::runtime_error("feature not found");
}
static Eigen::Vector3d ray(const json &f, std::size_t i) {
  const auto &v = f.at("corners_3d").at(i);
  Eigen::Vector3d out;
  out << v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>();
  return out;
}
static Eigen::Matrix3d cayley2rot(const Eigen::Vector3d &c) {
  const double scale = 1.0 + c.squaredNorm();
  Eigen::Matrix3d r;
  r << 1+c[0]*c[0]-c[1]*c[1]-c[2]*c[2], 2*(c[0]*c[1]-c[2]), 2*(c[0]*c[2]+c[1]),
       2*(c[0]*c[1]+c[2]), 1-c[0]*c[0]+c[1]*c[1]-c[2]*c[2], 2*(c[1]*c[2]-c[0]),
       2*(c[0]*c[2]-c[1]), 2*(c[1]*c[2]+c[0]), 1-c[0]*c[0]-c[1]*c[1]+c[2]*c[2];
  return r / scale;
}
static Eigen::Vector3d rot2cayley(const Eigen::Matrix3d &r) {
  Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d c = (r-i) * (r+i).inverse();
  return {-c(1,2), c(0,2), -c(0,1)};
}
static Vec residual(opengv::relative_pose::CentralRelativeAdapter &a, const Eigen::VectorXd &x) {
  const Eigen::Vector3d t = x.head<3>();
  const Eigen::Matrix3d r = cayley2rot(x.tail<3>());
  a.sett12(t); a.setR12(r);
  Vec f(a.getNumberCorrespondences());
  for (int i=0; i<f.size(); ++i) {
    Eigen::Matrix<double,4,1> p; p(3)=1.; p.head<3>()=opengv::triangulation::triangulate2(a,i);
    Eigen::Matrix<double,3,4> inv; inv.block<3,3>(0,0)=r.transpose(); inv.col(3)=-r.transpose()*t;
    const Eigen::Vector3d p1=p.head<3>(); const Eigen::Vector3d p2=inv*p;
    f(i)=1.-a.getBearingVector1(i).dot(p1/p1.norm())+1.-a.getBearingVector2(i).dot(p2/p2.norm());
  }
  return f;
}
static double scalar_dot3(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  double result = a[0] * b[0];
  result += a[1] * b[1];
  result += a[2] * b[2];
  return result;
}
static Vec residual_scalar_dots(opengv::relative_pose::CentralRelativeAdapter &a, const Eigen::VectorXd &x) {
  const Eigen::Vector3d t = x.head<3>();
  const Eigen::Matrix3d r = cayley2rot(x.tail<3>());
  a.sett12(t); a.setR12(r);
  Vec f(a.getNumberCorrespondences());
  for (int i=0; i<f.size(); ++i) {
    const Eigen::Vector3d point = opengv::triangulation::triangulate2(a,i);
    const Eigen::Vector3d second = r.transpose() * point - r.transpose() * t;
    const double n1 = std::sqrt(scalar_dot3(point, point));
    const double n2 = std::sqrt(scalar_dot3(second, second));
    const Eigen::Vector3d first_unit = point / n1;
    const Eigen::Vector3d second_unit = second / n2;
    f[i] = 1. - scalar_dot3(a.getBearingVector1(i), first_unit)
         + 1. - scalar_dot3(a.getBearingVector2(i), second_unit);
  }
  return f;
}
struct CustomQr {
  Mat qr;
  std::array<int, 6> pivots{};
  std::array<double, 6> householder{};
};
static CustomQr custom_qr(const Mat &jacobian) {
  const int rows = jacobian.rows(), cols = jacobian.cols();
  CustomQr out;
  out.qr = jacobian;
  std::vector<double> updated(cols), direct(cols);
  std::vector<int> ids(cols);
  for (int c = 0; c < cols; ++c) {
    double sum = 0.0;
    for (int r = 0; r < rows; ++r) sum += out.qr(r, c) * out.qr(r, c);
    updated[c] = std::sqrt(sum);
    direct[c] = updated[c];
    ids[c] = c;
  }
  const double downdate_threshold = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int k = 0; k < 6; ++k) {
    int biggest = k;
    double biggest_norm = updated[k];
    for (int c = k + 1; c < cols; ++c) {
      if (updated[c] > biggest_norm) { biggest = c; biggest_norm = updated[c]; }
    }
    if (biggest != k) {
      out.qr.col(k).swap(out.qr.col(biggest));
      std::swap(updated[k], updated[biggest]);
      std::swap(direct[k], direct[biggest]);
      std::swap(ids[k], ids[biggest]);
    }
    out.pivots[k] = ids[k];
    const double c0 = out.qr(k, k);
    double tail_sq_norm = 0.0;
    for (int r = k + 1; r < rows; ++r) tail_sq_norm += out.qr(r, k) * out.qr(r, k);
    const double tol = std::numeric_limits<double>::min();
    double tau = 0.0, beta = c0;
    if (!(tail_sq_norm <= tol && c0 * c0 <= tol)) {
      beta = std::sqrt(c0 * c0 + tail_sq_norm);
      if (c0 >= 0.0) beta = -beta;
      const double denominator = c0 - beta;
      for (int r = k + 1; r < rows; ++r) out.qr(r, k) /= denominator;
      tau = (beta - c0) / beta;
    }
    out.householder[k] = tau;
    out.qr(k, k) = beta;
    if (tau != 0.0 && k + 1 < cols) {
      for (int c = k + 1; c < cols; ++c) {
        double work = out.qr(k, c);
        for (int r = k + 1; r < rows; ++r) work += out.qr(r, k) * out.qr(r, c);
        out.qr(k, c) -= tau * work;
        for (int r = k + 1; r < rows; ++r) out.qr(r, c) -= tau * out.qr(r, k) * work;
      }
    }
    for (int c = k + 1; c < cols; ++c) {
      if (updated[c] != 0.0) {
        double temp = std::abs(out.qr(k, c)) / updated[c];
        temp = (1.0 + temp) * (1.0 - temp);
        temp = std::max(0.0, temp);
        const double ratio = updated[c] / direct[c];
        const double temp2 = temp * ratio * ratio;
        if (temp2 <= downdate_threshold) {
          double sum = 0.0;
          for (int r = k + 1; r < rows; ++r) sum += out.qr(r, c) * out.qr(r, c);
          updated[c] = std::sqrt(sum);
          direct[c] = updated[c];
        } else {
          updated[c] *= std::sqrt(temp);
        }
      }
    }
  }
  return out;
}
static Vec custom_qt_mul(const CustomQr &qr, const Vec &values) {
  Vec result = values;
  for (int k = 0; k < 6; ++k) {
    const double tau = qr.householder[k];
    if (tau == 0.0) continue;
    double work = result[k];
    for (int r = k + 1; r < result.size(); ++r) work += qr.qr(r, k) * result[r];
    result[k] -= tau * work;
    for (int r = k + 1; r < result.size(); ++r) result[r] -= tau * qr.qr(r, k) * work;
  }
  return result;
}
static void print_vec(const char *name, const Vec &v) { std::cout << name << " " << v.transpose() << "\n"; }
int main(int argc, char **argv) {
  std::ifstream rf(argv[1]), of(argv[2]); json raw, oracle; rf>>raw; of>>oracle;
  const std::uint64_t lf_id=1403636580113555456ULL, rf_id=1403636579763555584ULL;
  const auto &lf=feature(raw,lf_id,0), &rfv=feature(raw,rf_id,1);
  for (const auto &entry: oracle.at("bow_query").at("seeded_temporal_oracle")) {
    if (entry.at("left").at("frame_id")!=lf_id || entry.at("left").at("cam_id")!=0 || entry.at("right").at("frame_id")!=rf_id || entry.at("right").at("cam_id")!=1) continue;
    opengv::bearingVectors_t b1,b2;
    for (const auto &pair: entry.at("ransac_inlier_ids")) { b1.push_back(ray(lf,pair.at(0))); b2.push_back(ray(rfv,pair.at(1))); }
    opengv::relative_pose::CentralRelativeAdapter a(b1,b2);
    Eigen::Matrix3d r; for(int i=0;i<3;++i)for(int j=0;j<3;++j)r(i,j)=entry.at("ransac_model_rotation").at(i).at(j);
    Eigen::Vector3d t; for(int i=0;i<3;++i)t(i)=entry.at("ransac_model_translation").at(i);
    Eigen::VectorXd x(6); x.head<3>()=t; x.tail<3>()=rot2cayley(r);
    Vec f=residual(a,x); Vec fs=residual_scalar_dots(a,x); Mat j(f.size(),6);
    if (entry.at("seed") == 7) {
      std::cout << std::setprecision(17);
      a.setR12(cayley2rot(x.tail<3>())); a.sett12(x.head<3>());
      const Eigen::Vector3d p0 = opengv::triangulation::triangulate2(a, 0);
      const Eigen::Vector3d s0 = a.getR12().transpose() * p0 - a.getR12().transpose() * a.gett12();
      std::cout << "intermediate p0 " << p0.transpose() << " s0 " << s0.transpose() << "\n";
      const Eigen::Vector3d f1 = a.getBearingVector1(0), f2 = a.getBearingVector2(0);
      const Eigen::Vector3d fu = a.getR12() * f2;
      const double a00=f1.dot(f1), a10=f1.dot(fu), a01=-a10, a11=-fu.dot(fu);
      const double b0=a.gett12().dot(f1), b1=a.gett12().dot(fu);
      const double det=a00*a11-a10*a01, invdet=1.0/det;
      const double l0=(a11*invdet)*b0+(-a01*invdet)*b1;
      const double l1=(-a10*invdet)*b0+(a00*invdet)*b1;
      const Eigen::Vector3d xm=l0*f1, xn=a.gett12()+l1*fu;
      std::cout << "tri f1 " << f1.transpose() << " f2u " << fu.transpose()
                << " a " << a00 << ' ' << a10 << ' ' << a01 << ' ' << a11
                << " b " << b0 << ' ' << b1 << " det " << det
                << " lambda " << l0 << ' ' << l1 << " xm " << xm.transpose()
                << " xn " << xn.transpose() << "\n";
    }
    const double eps=std::sqrt(std::numeric_limits<double>::epsilon());
    for(int c=0;c<6;++c){ Eigen::VectorXd xp=x; const double h=std::max(eps*std::abs(x[c]),eps); xp[c]+=h; j.col(c)=(residual(a,xp)-f)/h; }
    Eigen::ColPivHouseholderQR<Mat> qr(j); Mat qrm=qr.matrixQR(); Vec qtf=f; qtf.applyOnTheLeft(qr.householderQ().adjoint());
    const CustomQr custom = custom_qr(j); const Vec custom_qtf = custom_qt_mul(custom, f);
    std::cout<<std::setprecision(17)<<"seed "<<entry.at("seed")<<" n "<<f.size()<<"\n"; print_vec("f",f); print_vec("fs",fs); print_vec("f_diff",fs-f); print_vec("colnorm",j.colwise().norm()); print_vec("qtf",qtf.head(6)); std::cout<<"piv "; for(int k=0;k<6;++k)std::cout<<qr.colsPermutation().indices()(k)<<' '; std::cout<<"\nqr\n"<<qrm.topLeftCorner(6,6)<<"\n";
    std::cout << "custom_piv "; for (int k=0;k<6;++k) std::cout << custom.pivots[k] << ' '; std::cout << "\n";
    std::cout << "max_qr_diff " << (custom.qr.topLeftCorner(6,6)-qrm.topLeftCorner(6,6)).cwiseAbs().maxCoeff() << " max_qtf_diff " << (custom_qtf.head(6)-qtf.head(6)).cwiseAbs().maxCoeff() << "\n";
  }
}
