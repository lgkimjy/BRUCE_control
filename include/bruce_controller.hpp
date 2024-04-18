/**
 * @file: bruce_controller.hpp
 * @brief: BRUCE Controller header file for the project
 * @author: Jun Young Kim
 */

#include <Eigen/Dense>
#include <iostream>

#include <mujoco/mujoco.h>

#include "Robot/robot.hpp"

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/spatial/se3.hpp>


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
    unsigned count_sim;

	Eigen::Matrix<double, 16, 1>	joint_torq;					//	Active joint torque
	Eigen::Matrix<double, 16, 1>	torq_ff;					//	Active joint torque
	Eigen::Matrix<double, 16, 1>	qpos_d, qvel_d, qacc_d;		//	Desired position & velocity of active joint
    
    // Gain Matrix
	Eigen::Matrix<double, 16, 16> 	K_qp, K_qv;					//	Joint gain matrices for active joint

	std::vector<Eigen::Vector3d>	                p_EE;       //	Position vector of i-th end-effector
    std::vector<Eigen::Vector3d>	                pdot_EE;    //	Linear velocity of i-th end-effector
    std::vector<Eigen::Matrix<double, 3, 22>>	    Jp_EE;	    //	i-th End-effector linear Jacobian
    std::vector<Eigen::Matrix<double, 3, 22>>	    Jr_EE;	    //	i-th End-effector angular Jacobian
    std::vector<Eigen::Matrix<double, 3, 22>>	    Jdotp_EE;	//	Time derivative of Jp_EE
    std::vector<Eigen::Matrix<double, 3, 22>>	    Jdotr_EE;	//	Time derivative of Jr_EE

public:
    // functions
    void initializeSystem();
    void control(mjModel* model, mjData* data);
    void getMuJoCoFeedback(mjData* data);
    void update();
    void computeJointLevelController(controllerTypeDef ctrlType);
};