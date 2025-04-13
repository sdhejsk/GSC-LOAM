#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
// #include <gtsam/nonlinear/derivatives.h> // 或者 <gtsam/geometry/Point2.h> 等其他路径


#include <boost/optional.hpp>
#include "Plane3.h"

using namespace gtsam;


template<typename T>
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
numericalDerivative(std::function<Eigen::VectorXd(const T&)> f, const T& x, double eps = 1e-5) {
    const int dim = f(x).size();
    const int N = traits<T>::dimension;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J(dim, N);
    T x_plus = x, x_minus = x;

    for (int i = 0; i < N; ++i) {
        Eigen::VectorXd delta = Eigen::VectorXd::Zero(N);
        delta(i) = eps;

        x_plus = traits<T>::Retract(x, delta);
        x_minus = traits<T>::Retract(x, -delta);

        J.col(i) = (f(x_plus) - f(x_minus)) / (2.0 * eps);
    }

    return J;
}

class PlaneAlignmentFactor : public NoiseModelFactor1<Pose3> {
 private:
  Plane3 nowPlane_;
  Plane3 planeNode_;  // 图中已有的地面节点

 public:
  PlaneAlignmentFactor(Key poseKey,
                       const Plane3& nowPlane,
                       const Plane3& planeNode,
                       const SharedNoiseModel& model)
      : NoiseModelFactor1<Pose3>(model, poseKey),
        nowPlane_(nowPlane),
        planeNode_(planeNode) {}

  // 平面 -> 只含 roll/pitch/z 的 Pose
  static Pose3 convertPlaneToPoseRPZ(const Plane3& plane) {
    Vector3 n = plane.normal();  // normalized normal
    Vector3 zAxis(0, 0, 1);
    
    boost::random::mt19937 rng;  // 创建随机数生成器
    Rot3 R = Rot3::AlignPair(Unit3(zAxis), Unit3(n), Unit3::Random(rng));  // 传递随机数生成器

    // 提取 roll pitch
    double roll, pitch, yaw;
    Vector3 rpy = R.rpy();
    roll = rpy(0); pitch = rpy(1); yaw = rpy(2);  // 修正：避免重复声明变量

    Rot3 R_rp = Rot3::RzRyRx(roll, pitch, 0.0);  // set yaw = 0

    double d = plane.d();
    Vector3 t(0, 0, -d);

    return Pose3(R_rp, t);
  }

  Vector evaluateError(const Pose3& pose,
    boost::optional<Eigen::MatrixXd&> H1 = boost::none
  ) const override {

    Pose3 t_w1 = convertPlaneToPoseRPZ(planeNode_);
    Pose3 t_w2 = convertPlaneToPoseRPZ(nowPlane_);

    Pose3 t1 = pose.inverse() * t_w1;
    Pose3 t2 = pose.inverse() * t_w2;

    Pose3 diff = t1.inverse() * t2;

    if (H1) {
      auto errorFunc = [&](const Pose3& pose) {
          return this->evaluateError(pose, boost::none);
      };
      *H1 = numericalDerivative<Pose3>(errorFunc, pose);
    }
  

    return Pose3::Logmap(diff);  // 修正：Logmap 返回的是 Vector 类型
  }

  virtual ~PlaneAlignmentFactor() {}
};




namespace gtsam {

  template<>
  struct traits<Plane3> {
    typedef Plane3 type;
    typedef type value_type;
  
    static void Print(const Plane3& m, const std::string& str = "") {
      std::cout << str << ": normal = " << m.normal().transpose() << ", d = " << m.d() << std::endl;
    }
  
    static bool Equals(const Plane3& m1, const Plane3& m2, double tol = 1e-8) {
      return (m1.normal() - m2.normal()).norm() < tol && std::abs(m1.d() - m2.d()) < tol;
    }
  
    static Eigen::VectorXd Local(const Plane3& origin, const Plane3& other) {
      Eigen::VectorXd v(4);
      v.head<3>() = other.normal() - origin.normal();
      v[3] = other.d() - origin.d();
      return v;
    }
  
    static Plane3 Retract(const Plane3& origin, const Eigen::VectorXd& v) {
      Eigen::Vector3d new_normal = origin.normal() + v.head<3>();
      double new_d = origin.d() + v[3];
      return Plane3(new_normal, new_d);
    }

    static size_t GetDimension(const Plane3&) { return dimension; }
  
    enum {
      dimension = 4
    };
  };
  
  }  // namespace gtsam
  
  
