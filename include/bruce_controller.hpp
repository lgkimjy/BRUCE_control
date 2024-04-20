/**
 * @file: bruce_controller.hpp
 * @brief: BRUCE Controller header file for the project
 * @author: Jun Young Kim
 */

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "Robot/robot.hpp"
#include "Trajectory/JointTrajectory.h"
// #include "WBC/wbc_base.hpp"
#include "WBC/weighted_wbc.hpp"

typedef enum{
  JOINT_PD			= 0X00, // Joint PD
  TORQ  	   		= 0X01, // Purly Inverse Dynamics calculated Joint torque
  JOINT_PD_TORQ    	= 0X02, // Joint PD + feedforward Joint torque
  CARTESIAN_PD_TORQ	= 0X03, // Cartesain PD + feedforward Joint torque
}controllerTypeDef;

constexpr int CONTROL_RATE = 1;

class BRUCEController {
private:
public:
    BRUCEController();
    ~BRUCEController();

    Robot bruce;

    double sim_time;
    int    tmp = 0;
    unsigned count_sim;

	Eigen::Matrix<double, ACTIVE_DOF, 1>	        joint_torq;					       //	Active joint torque
	Eigen::Matrix<double, ACTIVE_DOF, 1>	        torq_ff;					       //	Active joint torque
	Eigen::Matrix<double, ACTIVE_DOF, 1>	        qpos_d, qvel_d, qacc_d;		       //	Desired position & velocity of active joint
	Eigen::Matrix<double, ACTIVE_DOF, 1>	        qpos_ref, qvel_ref, qacc_ref;	    //	Desired position & velocity of active joint
    
    // Gain Matrix
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	Kp_q, Kd_q;					//	Joint gain matrices for active joint
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	Kp_qcmd, Kd_qcmd;			//	Joint gain matrices for active joint
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	Kp_swing, Kd_swing;			//	Joint gain matrices for active joint


    // End-effector related variables
    std::vector<std::string>	                            EEnames;    //	End-effector names
	std::vector<Eigen::Vector3d>	                        p_EE;       //	Position vector of i-th end-effector
	std::vector<Eigen::Matrix3d>	                        R_EE;       //	Position vector of i-th end-effector
    std::vector<Eigen::Vector3d>	                        pdot_EE;    //	Linear velocity of i-th end-effector
    std::vector<Eigen::Vector3d>	                        omega_EE;   //	Angular velocity of i-th end-effector
    std::vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>	    Jp_EE;	    //	i-th End-effector linear Jacobian
    std::vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>	    Jr_EE;	    //	i-th End-effector angular Jacobian
    std::vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>	    Jdotp_EE;	//	Time derivative of Jp_EE
    std::vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>	    Jdotr_EE;	//	Time derivative of Jr_EE

	CP2P_Traj<ACTIVE_DOF, sysReal> 			Joint_Traj;

    std::shared_ptr<wbcBase> wbc;

public:
    // functions
    void initializeSystem();
    void readConfig();
    void control(mjModel* model, mjData* data);
    void getMuJoCoFeedback(mjData* data);
    void JointPlanner();
    void JointPlanner2();
    void computeEEKinematics();
    void updateControlAlgorithm();
    
    void computeCoMPosTask();
    void computeBaseOrientationTask();
    void computeJointLevelController(controllerTypeDef ctrlType);
};