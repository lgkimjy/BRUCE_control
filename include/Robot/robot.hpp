#include <iostream>

#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include "pinocchio/algorithm/rnea.hpp"         // rnea: recursive Newton-Euler algorithm

class Robot {
private:
public:
    Robot();
    ~Robot();

    pinocchio::Model            model_;
    pinocchio::Data             data_;
    double                      nv_;
    double                      nq_;
    double                      gravity_;
    std::vector<std::string>    jnames_;

    Eigen::Matrix<double, 22, 1> qpos, qvel, qacc; // Joint position, velocity, acceleration

    double              mass;
    Eigen::Vector3d     p_com;
    Eigen::Vector3d     v_com;
    Eigen::Vector3d     a_com;

    Eigen::Vector3d     p_B;
    Eigen::Vector4d     quat_B;
    Eigen::Matrix3d     R_B;

    Eigen::Vector3d     pdot_B;
    Eigen::Vector3d     omega_B;
    Eigen::Vector3d     varphi_B;

    Eigen::Matrix<double, 16, 1>    q;
    Eigen::Matrix<double, 16, 1>    qdot;

    Eigen::Matrix<double, 23, 1>    xi_quat;
    Eigen::Matrix<double, 22, 1>    xidot;
    Eigen::Matrix<double, 22, 1>    xidot_tmp;
    Eigen::Matrix<double, 22, 1>    xiddot;

public:
    void loadRobotModel();
    void computeForwardKinematics();
    void computeInverseKinematics();
    void computeInverseDynamics();
};