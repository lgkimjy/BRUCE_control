#include "bruce_controller.hpp"

BRUCEController::BRUCEController() :count_sim(0)
{
    std::cout << "BRUCEController Constructor" << std::endl;

    joint_torq.setZero();
    K_qp.setIdentity();
    K_qv.setIdentity();
}

BRUCEController::~BRUCEController() 
{
    std::cout << "BRUCEController Destructor" << std::endl;
}

void BRUCEController::initializeSystem()
{
    bruce.loadRobotModel();
}

void BRUCEController::control(mjModel* model, mjData* data)
{
	if (count_sim % CONTROL_RATE == 0) 
    {
        getMuJoCoFeedback(data);
        bruce.computeForwardKinematics();
        bruce.computeInverseDynamics();
        update();

        for(int i=0; i<16; i++) {
            data->ctrl[i] = joint_torq(i);
        }
    }
	++count_sim;
}

void BRUCEController::getMuJoCoFeedback(mjData* data)
{
	/////	Position vector of floating-base body w.r.t {I}
	bruce.p_B(0) = data->qpos[0];
	bruce.p_B(1) = data->qpos[1];
	bruce.p_B(2) = data->qpos[2];

	/////	Orientation of floating-base body w.r.t {I}
	bruce.quat_B(0) = data->qpos[3];
	bruce.quat_B(1) = data->qpos[4];
	bruce.quat_B(2) = data->qpos[5];	
	bruce.quat_B(3) = data->qpos[6];

	// _Quat2Rot(robot.quat_B);
    Eigen::Quaterniond tmp_quat;
    tmp_quat.w() = bruce.quat_B(0);
    tmp_quat.x() = bruce.quat_B(1);
    tmp_quat.y() = bruce.quat_B(2);
    tmp_quat.z() = bruce.quat_B(3);
    bruce.R_B = tmp_quat.toRotationMatrix();

	/////	Linear velocity of floating-base body w.r.t {I}
	bruce.pdot_B(0) = data->qvel[0];
	bruce.pdot_B(1) = data->qvel[1];
	bruce.pdot_B(2) = data->qvel[2];

	/////	Angular velocity of floating-base body expressed in {B}
	bruce.varphi_B(0) =data->qvel[3];
	bruce.varphi_B(1) =data->qvel[4];
	bruce.varphi_B(2) =data->qvel[5];

	/////	Convert to absolute angular velocity
	bruce.omega_B = bruce.R_B * bruce.varphi_B;

	/////	Set generalized coordinates
	bruce.xi_quat.segment<3>(0) = bruce.p_B;
	bruce.xi_quat.segment<4>(3) = bruce.quat_B;

	bruce.xidot.segment<3>(0) = bruce.pdot_B;
	bruce.xidot.segment<3>(3) = bruce.omega_B;

	for (int i = 0; i < 16; i++) {
		// robot.q(i) = data->qpos[DOF_BASEBODY_QUAT + i];
		// robot.qdot(i) = data->qvel[DOF_BASEBODY + i];
        bruce.q(i) = data->qpos[7 + i];
		bruce.qdot(i) = data->qvel[6 + i];
	}

	// /////	Set joint coordinates and joint velocity
	// robot.xi_quat.segment(DOF_BASEBODY_QUAT, ACTIVE_DOF) = bruce.q;
	// robot.xidot.segment(DOF_BASEBODY, ACTIVE_DOF) = bruce.qdot;

	// /////	Compute joint acceleration by numerical diff.
	// robot.xiddot = (robot.xidot - robot.xidot_tmp) / robot.getSamplingTime();
	// robot.xidot_tmp = robot.xidot;

	sim_time = data->time;
}

void BRUCEController::update()
{
    // std::cout << bruce.nv_ << std::endl;
    // std::cout << bruce.nq_ << std::endl;

    K_qp = 100 * K_qp;
    K_qv = 15 * K_qv;
    qpos_d.setZero();
    qvel_d.setZero();
    computeJointLevelController(JOINT_PD);
    // joint_torq.setOnes();
    // std::cout << "bruce q_d: "  << qpos_d.transpose() << std::endl;
    // std::cout << "bruce q: "    << bruce.q.transpose() << std::endl;
    // std::cout << "joint_torq: " << joint_torq.transpose() << std::endl << std::endl;
}

void BRUCEController::computeJointLevelController(controllerTypeDef ctrlType)
{
    switch (ctrlType)
    {
    case JOINT_PD:
        joint_torq = K_qp * (qpos_d - bruce.q) + K_qv * (qvel_d - bruce.qdot);
        break;
    case TORQ:
        joint_torq = torq_ff;
        break;
    case JOINT_PD_TORQ:
        joint_torq = K_qp * (qpos_d - bruce.q) + K_qv * (qvel_d - bruce.qdot) + torq_ff;
        break;
    case CARTESIAN_PD_TORQ:
        break;
    default:
        break;
    }
}