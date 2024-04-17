#include "main.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

int main() {
    std::cout << "Hello, World!" << std::endl;

    Eigen::Vector3d v;
    v << 1, 2, 3;
    Eigen::Matrix3d R =  Eigen::AngleAxisd(1.0471975512, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pinocchio::SE3 A(R, v);
    std::cout << A << std::endl;

    Eigen::Vector3d mulipler;
    mulipler << 1, 1, 1;

    std::cout << A.actInv(mulipler).transpose() << std::endl;
    std::cout << (A.rotation().transpose() * mulipler).transpose() << std::endl;
    std::cout << (R.transpose() * mulipler).transpose() << std::endl;

    std::cout << A.act(mulipler).transpose() << std::endl;

    BRUCEController controller;

    return 0;
}