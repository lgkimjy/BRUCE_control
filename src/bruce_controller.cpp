/**
 * @file: brcue_controller.cpp
 * @brief: Controller for the BRUCE robot using mujoco and pinocchio
 * @author: Jun Young Kim
 */

#include "bruce_controller.hpp"

BRUCEController::BRUCEController() :count_sim(0)
{
    // std::cout << "BRUCEController Constructor" << std::endl;

    readConfig();

    qpos_d.setZero();
    qvel_d.setZero();
    joint_torq.setZero();    

    // End-effector related variables initialization
    p_EE.reserve(EEnames.size());
    R_EE.reserve(EEnames.size());
    pdot_EE.reserve(EEnames.size());
    omega_EE.reserve(EEnames.size());
    Jp_EE.reserve(EEnames.size());
    Jr_EE.reserve(EEnames.size());
    Jdotp_EE.reserve(EEnames.size());
    Jdotr_EE.reserve(EEnames.size());

    for (int i = 0; i < EEnames.size(); i++) {
		p_EE[i].setZero();
        R_EE[i].setIdentity();
		pdot_EE[i].setZero();
		omega_EE[i].setZero();
		Jp_EE[i].setZero();
		Jr_EE[i].setZero();
		Jdotp_EE[i].setZero();
		Jdotr_EE[i].setZero();
	}
}

BRUCEController::~BRUCEController() 
{
    std::cout << "BRUCEController Destructor" << std::endl;
}

void BRUCEController::initializeSystem()
{
    bruce.loadRobotModel();
    wbc = std::make_shared<WeightedWBC>();
}

void BRUCEController::readConfig()
{
	YAML::Node yaml_node;
	std::string path = CMAKE_SOURCE_DIR "/config/config.yaml";
	try
	{
		yaml_node = YAML::LoadFile(path.c_str());

        // Gain Initialization
        Kp_q.setIdentity();
        Kd_q.setIdentity();
        Kp_qcmd.setIdentity();
        Kd_qcmd.setIdentity();
        Kp_swing.setIdentity();
        Kd_swing.setIdentity();

		for (int i=0; i<ACTIVE_DOF; i++) {
			Kp_qcmd(i, i) = yaml_node["qcmd_gain"]["kp"][i].as<double>();
			Kd_qcmd(i, i) = yaml_node["qcmd_gain"]["kd"][i].as<double>();
			Kp_q(i, i) = yaml_node["joint_level_gain"]["kp"][i].as<double>();
			Kd_q(i, i) = yaml_node["joint_level_gain"]["kd"][i].as<double>();
            qpos_ref(i) = D2R(yaml_node["initial_joint_pos"][i].as<double>());
		}
        for(int i=0; i<yaml_node["end_effector"].size(); i++) {
            std::string frame_name = yaml_node["end_effector"][i].as<std::string>();
            EEnames.push_back(frame_name);
        }
		std::cout << GREEN << "Controller Gain Config YAML File Loaded!" << RESET << std::endl;
	}
	catch(const std::exception& e)
	{
		std::cerr << RED << "Fail to read Controller Gain Config YAML file" << RESET << std::endl;
		exit(0);
	}
}

void BRUCEController::control(mjModel* model, mjData* data)
{
	if (count_sim % CONTROL_RATE == 0) 
    {
        getMuJoCoFeedback(data);            // get feedback from mujoco about the BRUCE, compute base on .xml model
        bruce.computeForwardKinematics();
        bruce.computeDynamics();
        bruce.computeCoMMotion();
        computeEEKinematics();
        updateControlAlgorithm();                           // run proposed methods/algorithm

        for(int i=0; i<ACTIVE_DOF; i++) {
            data->ctrl[i] = joint_torq(i);  // set control torque to mujoco
        }
    }
	++count_sim;
}

void BRUCEController::getMuJoCoFeedback(mjData* data)
{
	sim_time = data->time;

	/////	Position vector of floating-base body w.r.t {I}
	bruce.p_B_(0) = data->qpos[0];
	bruce.p_B_(1) = data->qpos[1];
	bruce.p_B_(2) = data->qpos[2];

	/////	Orientation of floating-base body w.r.t {I} 
    /////   MuJoCo uses quaternion as w,x,y,z, but pinocchio uses x,y,z,w
	bruce.quat_B_(0) = data->qpos[4];   // x
	bruce.quat_B_(1) = data->qpos[5];   // y
	bruce.quat_B_(2) = data->qpos[6];	// z
	bruce.quat_B_(3) = data->qpos[3];   // w

	// _Quat2Rot(robot.quat_B);
    Eigen::Quaterniond tmp_quat;
    tmp_quat.w() = bruce.quat_B_(3);
    tmp_quat.x() = bruce.quat_B_(0);
    tmp_quat.y() = bruce.quat_B_(1);
    tmp_quat.z() = bruce.quat_B_(2);
    bruce.R_B_ = tmp_quat.toRotationMatrix();

	/////	Linear velocity of floating-base body w.r.t {I}
	bruce.pdot_B_(0) = data->qvel[0];
	bruce.pdot_B_(1) = data->qvel[1];
	bruce.pdot_B_(2) = data->qvel[2];

	/////	Angular velocity of floating-base body expressed in {B}
	bruce.varphi_B_(0) =data->qvel[3];
	bruce.varphi_B_(1) =data->qvel[4];
	bruce.varphi_B_(2) =data->qvel[5];

	/////	Convert to absolute angular velocity
	bruce.omega_B_ = bruce.R_B_ * bruce.varphi_B_;

	/////	Set generalized coordinates
	bruce.xi_quat_.segment<DOF3>(0) = bruce.p_B_;
	bruce.xi_quat_.segment<DOF4>(DOF3) = bruce.quat_B_; 

	bruce.xidot_.segment<DOF3>(0) = bruce.pdot_B_;
	bruce.xidot_.segment<DOF3>(DOF3) = bruce.omega_B_;

	for (int i = 0; i < ACTIVE_DOF; i++) {
        bruce.q_(i) = data->qpos[DOF_BASEBODY_QUAT + i];
		bruce.qdot_(i) = data->qvel[DOF_BASEBODY + i];
	}

	// /////	Set joint coordinates and joint velocity
	bruce.xi_quat_.segment(DOF_BASEBODY_QUAT, ACTIVE_DOF) = bruce.q_;
	bruce.xidot_.segment(DOF_BASEBODY, ACTIVE_DOF) = bruce.qdot_;

	// /////	Compute joint acceleration by numerical diff.
	bruce.xiddot_ = (bruce.xidot_ - bruce.xidot_tmp_) / 0.001; //bruce.getSamplingTime();
	bruce.xidot_tmp_ = bruce.xidot_;
}

void BRUCEController::computeEEKinematics()
{
    for(int i(0); i<EEnames.size(); i++) 
    {
        pinocchio::Model::FrameIndex link_number = bruce.model_.getFrameId(EEnames[i]);
        p_EE[i] = bruce.data_.oMf[link_number].translation();
        R_EE[i] = bruce.data_.oMf[link_number].rotation();

        pinocchio::Data::Matrix6x J(DOF6, TOTAL_DOF);
        J.fill(0);
        pinocchio::getFrameJacobian(bruce.model_, bruce.data_, link_number, pinocchio::LOCAL, J);
        // Jp_EE[i] = J.topRows(3);
        // Jr_EE[i] = J.bottomRows(3);
        Jp_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * J.topRows(3);
        Jr_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * J.bottomRows(3);
        // Rwb * (data_->oMf[link_number].rotation()) * J.topRows(3);
        // Rwb * (data_->oMf[link_number].rotation()) * J.bottomRows(3);
    }
}

void BRUCEController::JointPlanner()
{
	if(Joint_Traj.is_moving_ == false)
	{
		Joint_Traj.setTargetPosition(bruce.q_, qpos_ref, 2.0, 1 / 1000.0, QUINTIC);
	}
	Joint_Traj.computeTraj(qpos_d, qvel_d, qacc_d);
}

void BRUCEController::JointPlanner2()
{
	if(Joint_Traj.is_moving_ == false)
	{
        qpos_ref(10) = D2R(30 * pow(-1, tmp));
        qpos_ref(13) = D2R(30 * pow(-1, tmp));
		Joint_Traj.setTargetPosition(bruce.q_, qpos_ref, 2.0, 1 / 1000.0, QUINTIC);
        tmp += 1;
	}
	Joint_Traj.computeTraj(qpos_d, qvel_d, qacc_d);
}

void BRUCEController::updateControlAlgorithm()
{
    if(sim_time < 1.0) {
        // torque ON at initial joint poisition
        computeJointLevelController(JOINT_PD);
    }
    else if(sim_time < 3.0) {
        // squat motion or initial pose motion based on Joint PD control
        JointPlanner();
        computeJointLevelController(JOINT_PD);
    }
    else {
        JointPlanner2();
        // WBC starts here
        Eigen::VectorXd decision_var_tmp;
        decision_var_tmp = wbc->update();

        /*  segment decision_var_tmp */ 
        // qddot_cmd = decision_var_tmp.segment<ACTIVE_DOF>(0);
        // grf_des = decision_var_tmp.segment<DOF3>(ACTIVE_DOF);
        // torq_ff = bruce.computeInverseDynamics(xi_quat_cmd, xidot_cmd, xiddot_cmd);
        computeJointLevelController(JOINT_PD);
    }
}

void BRUCEController::computeJointLevelController(controllerTypeDef ctrlType)
{
    switch (ctrlType)
    {
    case JOINT_PD:
        joint_torq = Kp_q * (qpos_d - bruce.q_) + Kd_q * (qvel_d - bruce.qdot_);
        break;
    case TORQ:
        joint_torq = torq_ff;
        break;
    case JOINT_PD_TORQ:
        joint_torq = Kp_q * (qpos_d - bruce.q_) + Kd_q * (qvel_d - bruce.qdot_) + torq_ff;
        break;
    case CARTESIAN_PD_TORQ:
        joint_torq = Kp_swing * (qpos_d - bruce.q_) + Kd_swing * (qvel_d - bruce.qdot_) + torq_ff;
        break;
    default:
        break;
    }
}