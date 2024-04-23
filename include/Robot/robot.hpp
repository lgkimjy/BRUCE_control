/**
 * @file: robot.hpp
 * @brief: Robot KinDyn Computation Using Pinocchio
 * @author: Jun Young Kim
 */

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>

/* pinocchio related headers */
#include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/fwd.hpp>                 // forward declarations must be included first.
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/center-of-mass-derivatives.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>         // rnea: recursive Newton-Euler algorithm

#include "Robot/bruce.hpp"
#include "utils.hpp"

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
    double                      mass_;

    Eigen::Matrix<double, TOTAL_DOF, 1>         qpos_, qvel_, qacc_; // Joint position, velocity, acceleration
    Eigen::Matrix<double, ACTIVE_DOF, 1>        torq_;         // Joint torque
    
    Eigen::Vector3d                             p_CoM_;
    Eigen::Vector3d                             pdot_CoM_;
    Eigen::Vector3d                             pddot_CoM_;
    Eigen::Matrix<double, 3, 22>                J_CoM_;

    Eigen::Vector3d                             p_B_;
    Eigen::Vector4d                             quat_B_;
    Eigen::Matrix3d                             R_B_;

    Eigen::Vector3d                             pdot_B_;
    Eigen::Vector3d                             omega_B_;
    Eigen::Vector3d                             varphi_B_;

    Eigen::Matrix<double, ACTIVE_DOF, 1>        q_;
    Eigen::Matrix<double, ACTIVE_DOF, 1>        qdot_;

    Eigen::Matrix<double, TOTAL_DOF_QUAT, 1>    xi_quat_;
    Eigen::Matrix<double, TOTAL_DOF, 1>         xidot_;
    Eigen::Matrix<double, TOTAL_DOF, 1>         xidot_tmp_;
    Eigen::Matrix<double, TOTAL_DOF, 1>         xiddot_;

	//////////////////////////////////////////////////////////////////////
	/////	Dynamics Related Terms : Inertial frame-based
	//////////////////////////////////////////////////////////////////////
	Eigen::Matrix<double, TOTAL_DOF, 1>				g_vec_;		//	Gravity force vector
	Eigen::Matrix<double, ACTIVE_DOF, 1>			I_actuator_;	//	Added joint inertia for geared actuator

	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>		T_B_;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>		M_mat_;		//	Joint space inertia matrix
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>		C_mat_;		//	Coriolis & Centrifugal matrix
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>		Mdot_mat_;	//	Derivative of joint space inertia matrix

	Eigen::Matrix<double, DOF3, TOTAL_DOF>			M_p_;		//	Linear part of inertia matrix = J_CoM !!
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			M_o_;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			C_p_;		//	Linear part of Coriolis matrix = Jdot_CoM !!
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			C_o_;

	/////	Centroidal Momentum Dynamics
	Eigen::Vector3d									l_b_, k_b_;			    //	Linear & angular momentum @{B}
	Eigen::Vector3d 								l_CoM_, k_CoM_;		    //	Linear & angular momentum @CoM
	Eigen::Vector3d 								ldot_CoM_, kdot_CoM_;   //	Time derivative of linear & angular momentum @ CoM
	Eigen::Matrix<double, DOF6, 1> 					h_b_;				    //	Momentum w.r.t. {B} expressed in {I}
	Eigen::Matrix<double, DOF6, 1> 					h_CoM_;				    //	Centroidal momentum expressed in {I}
	Eigen::Matrix<double, DOF6, 1> 					hdot_CoM_;			    //	Time derivative of h_CoM

	Eigen::Matrix<double, DOF3, TOTAL_DOF>			Ap_CoM_;		        //	Centroidal momentum matrix (CMM) - Linear
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			Ar_CoM_;		        //	Centroidal momentum matrix (CMM) - Angular
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			Adotp_CoM_;	            //	Time derivative of Ap_CoM
	Eigen::Matrix<double, DOF3, TOTAL_DOF>			Adotr_CoM_;	            //	Time derivative of Ar_CoM

	Eigen::Matrix<double, DOF6, TOTAL_DOF>			A_CoM_;		            //	Centroidal momentum matrix (CMM)
	Eigen::Matrix<double, DOF6, TOTAL_DOF>			Adot_CoM_;	            //	Time derivative of CMM

public:
    void loadRobotModel();
    void computeForwardKinematics();
    void computeInverseKinematics();
    void computeDynamics();
    void computeCoMMotion();
    void computeEEKinemaitcs();
    Eigen::VectorXd computeInverseDynamics(Eigen::VectorXd xi_quat_cmd, Eigen::VectorXd xidot_cmd, Eigen::VectorXd xiddot_cmd);
};