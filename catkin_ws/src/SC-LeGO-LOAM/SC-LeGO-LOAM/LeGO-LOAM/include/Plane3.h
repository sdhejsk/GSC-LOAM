#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>
#include <gtsam/base/Manifold.h>

namespace gtsam {

class Plane3 {
private:
  Eigen::Vector3d normal_;
  double d_;

public:
  Plane3() : normal_(Eigen::Vector3d::UnitZ()), d_(0.0) {}
  Plane3(const Eigen::Vector3d& n, double d) : normal_(n.normalized()), d_(d) {}

  const Eigen::Vector3d& normal() const { return normal_; }
  double d() const { return d_; }

  // 将平面从世界坐标系变换到某个位姿下的局部坐标系
  Plane3 transform(const Pose3& pose) const {
    Eigen::Vector3d n_local = pose.rotation().transpose() * normal_;
    double d_local = d_ - n_local.dot(pose.translation());
    return Plane3(n_local, d_local);
  }

  // 转换为仿射矩阵形式
  Eigen::Vector4d coefficients() const {
    return Eigen::Vector4d(normal_(0), normal_(1), normal_(2), d_);
  }
};

  


}  // namespace gtsam
