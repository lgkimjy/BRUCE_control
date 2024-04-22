/**
 * @file: brcue_controller.cpp
 * @brief: Controller for the BRUCE robot using mujoco and pinocchio
 * @author: Jun Young Kim
 */

#include "bruce_controller.hpp"

BRUCEController::BRUCEController() :count_sim(0), count_ctrl(0)
{
    // std::cout << "BRUCEController Constructor" << std::endl;

    readConfig();

    qpos_d.setZero();
    qvel_d.setZero();
    joint_torq.setZero();    

    // End-effector related variables initialization
    p_EE.reserve(EEnames.size());
    R_EE.reserve(EEnames.size());
    T_EE.reserve(EEnames.size());
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

    p_contact.reserve(4);
    R_contact.reserve(4);
    pdot_contact.reserve(4);
    Jp_contact.reserve(4);
    Jr_contact.reserve(4);
    Jdotp_contact.reserve(4);
    Jdotr_contact.reserve(4);
    for(int i=0; i<4; i++) {
        p_contact[i].setZero();
        R_contact[i].setIdentity();
        pdot_contact[i].setZero();
        Jp_contact[i].setZero();
        Jr_contact[i].setZero();
        Jdotp_contact[i].setZero();
        Jdotr_contact[i].setZero();
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

        offset.reserve(4);

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
        for(int i=0; i<4; i++) 
        {
            offset[0](i) = yaml_node["contact_offset"]["LeftToe"][i].as<double>();
            offset[1](i) = yaml_node["contact_offset"]["LeftHeel"][i].as<double>();
            offset[2](i) = yaml_node["contact_offset"]["RightToe"][i].as<double>();
            offset[3](i) = yaml_node["contact_offset"]["RightHeel"][i].as<double>();
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
    compareModelComputation(model, data, count_sim);

	if (count_sim % CONTROL_RATE == 0) 
    {
        getMuJoCoFeedback(data);            // get feedback from mujoco about the BRUCE, compute base on .xml model
        bruce.computeForwardKinematics();
        bruce.computeDynamics();
        bruce.computeCoMMotion();
        computeEEKinematics();              // compute End-Effector Kinematics (position, velocity, Jacobian, etc.)
        computeContactKinematics();         // compute Contact Kinematics
        updateControlAlgorithm();           // run proposed methods/algorithm

        for(int i=0; i<ACTIVE_DOF; i++) {
            data->ctrl[i] = joint_torq(i);  // set control torque to mujoco
        }
		++count_ctrl;
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
        p_EE[i] = bruce.data_.oMf[link_number].translation();           // + Eigen::Vector3d{0.055, 0, -0.025};
        R_EE[i] = bruce.data_.oMf[link_number].rotation();
        T_EE[i] = bruce.data_.oMf[link_number].toHomogeneousMatrix();

        pinocchio::Data::Matrix6x J(DOF6, TOTAL_DOF);
        pinocchio::Data::Matrix6x Jdot(DOF6, TOTAL_DOF);
        J.fill(0);
        Jdot.fill(0);
        pinocchio::getFrameJacobian(bruce.model_, bruce.data_, link_number, pinocchio::LOCAL, J);                       // LOCAL, WORLD, LOCAL_WORLD_ALIGNED ?
        pinocchio::getFrameJacobianTimeVariation(bruce.model_, bruce.data_, link_number, pinocchio::LOCAL, Jdot);
        // Jp_EE[i] = J.topRows(3);
        // Jr_EE[i] = J.bottomRows(3);
        Jp_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * J.topRows(3);
        Jr_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * J.bottomRows(3);
        std::cout << "Jp_EE["<<i<<"]" << std::endl << Jp_EE[i] << std::endl;
        std::cout << "Jp_EE["<<i<<"]" << std::endl << Jr_EE[i] << std::endl;
        // Rwb * (data_->oMf[link_number].rotation()) * J.topRows(3);
        // Rwb * (data_->oMf[link_number].rotation()) * J.bottomRows(3);
        Jdotp_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * Jdot.topRows(3);
        Jdotr_EE[i] = bruce.R_B_ * (bruce.data_.oMf[link_number].rotation()) * Jdot.bottomRows(3);
    }
}

void BRUCEController::computeContactKinematics()
{
    for(int i=0; i<2; i++) 
    {
        p_contact[i] = (T_EE[0] * offset[i]).segment<3>(0);
        R_contact[i] = R_EE[0];
        pinocchio::Data::Matrix6x J(DOF6, TOTAL_DOF);
        J.fill(0);
    }
    for(int i=2; i<4; i++) 
    {
        p_contact[i] = (T_EE[1] * offset[i]).segment<3>(0);
        R_contact[i] = R_EE[1];
        pinocchio::Data::Matrix6x J(DOF6, TOTAL_DOF);
        J.fill(0);
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



Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_EE_mj[NO_OF_EE], Pre_Jr_EE_mj[NO_OF_EE];
// Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnk_mj[NO_OF_BODY], Pre_Jr_lnk_mj[NO_OF_BODY];
// Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnkCoM_mj[NO_OF_BODY];
void BRUCEController::compareModelComputation(const mjModel* uModel, mjData* uData, const int& count)
{
	int i, j, k;
	int error = 0;
	int mjBodyID;

	Eigen::Vector3d temp_vec;
	Eigen::Vector4d temp_quat;

	//////////	Get body ID for end-effectors (defined in XML file via model->site !)
	int no_of_EE = uModel->nsite;
    double dT = uModel->opt.timestep;
	std::vector<int> id_body_EE;
	for (i = 0; i < no_of_EE; i++) {
		id_body_EE.push_back(uModel->site_bodyid[i] - 1);
	}
	id_body_EE.shrink_to_fit();

	int start_BodyID = 1;

	sysReal error_precision = uModel->opt.tolerance;

	mjtNum* jacp = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* jacr = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* dense_M = new mjtNum[TOTAL_DOF * TOTAL_DOF]{ 0 };
	mjtNum* temp_vec3 = new mjtNum[DOF3]{ 0 };

	// Eigen::Vector3d								p_EE[NO_OF_EE];
	// Eigen::Matrix3d								R_EE[NO_OF_EE];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE[NO_OF_EE];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE[NO_OF_EE];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE[NO_OF_EE];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE[NO_OF_EE];

	Eigen::Vector3d								p_EE_mj[NO_OF_EE];
	Eigen::Matrix3d								R_EE_mj[NO_OF_EE];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE_mj[NO_OF_EE];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE_mj[NO_OF_EE];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE_mj[NO_OF_EE];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE_mj[NO_OF_EE];

	// Eigen::Vector3d								p_lnk[NO_OF_BODY];
	// Eigen::Vector3d								p_lnkCoM[NO_OF_BODY];
	// Eigen::Matrix3d								R_lnk[NO_OF_BODY];
	// Eigen::Matrix3d								R_lnkCoM[NO_OF_BODY];

	// Eigen::Vector3d								p_lnk_mj[NO_OF_BODY];
	// Eigen::Vector3d								p_lnkCoM_mj[NO_OF_BODY];
	// Eigen::Matrix3d								R_lnk_mj[NO_OF_BODY];
	// Eigen::Matrix3d								R_lnkCoM_mj[NO_OF_BODY];

	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk[NO_OF_BODY];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk[NO_OF_BODY];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_lnkCoM[NO_OF_BODY];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk[NO_OF_BODY];
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk[NO_OF_BODY];

	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk_mj[NO_OF_BODY];		//	Linear Jacobian of i-th link in {I}
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk_mj[NO_OF_BODY];		//	Angular Jacobian of i-th link in {I}
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk_mj[NO_OF_BODY];	//	Time derivative of Jp_lnk
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk_mj[NO_OF_BODY];	//	Time derivative of Jr_lnk
	// Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnkCoM_mj[NO_OF_BODY];

	// Eigen::Matrix<double, TOTAL_DOF, 1>			hvec;
	// Eigen::Matrix<double, TOTAL_DOF, 1>			hvec_mj;
	// Eigen::Matrix<double, TOTAL_DOF, 1>			gvec_mj;
	// Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>	Mmat_mj;

	Eigen::Vector3d								p_G;
	Eigen::Vector3d								p_CoM_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_G;

	// Eigen::Vector3d								tempVec3_1, tempVec3_2, tempVec3_3, tempVec3_4;
	// Eigen::Vector3d								tmpPosVec1, tmpPosVec2;
	// Eigen::Matrix3d								TmpRot_Mat1, TmpRot_Mat2;
	
    Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> X_O;

	for(int i = 0; i<NO_OF_EE; i++) {
		// p_EE[i].setZero();
		// R_EE[i].setZero();
		// Jp_EE[i].setZero();
		// Jr_EE[i].setZero();
		// Jdotp_EE[i].setZero();
		// Jdotr_EE[i].setZero();
		p_EE_mj[i].setZero();
		R_EE_mj[i].setZero();
		Jp_EE_mj[i].setZero();
		Jr_EE_mj[i].setZero();
		Jdotp_EE_mj[i].setZero();
		Jdotr_EE_mj[i].setZero();
	}
    X_O.setIdentity();
    X_O.block<3, 3>(3, 3) = bruce.R_B_;


	/////	06. CoM Position of the system from MuJoCo !
	p_CoM_mj(0) = uData->subtree_com[0 * DOF3];
	p_CoM_mj(1) = uData->subtree_com[0 * DOF3 + 1];
	p_CoM_mj(2) = uData->subtree_com[0 * DOF3 + 2];

	/////	07. CoM Jacobian of the system from MuJoCo
	mj_jacSubtreeCom(uModel, uData, jacp, start_BodyID);
	for (j = 0; j < DOF3; j++) {
		for (k = 0; k < TOTAL_DOF; k++) {
			J_G_mj(j, k) = jacp[j * TOTAL_DOF + k];
		}
	}
	//	Transform from MuJoCo coordinate to inertial coordinate
	J_G_mj = J_G_mj * Eigen::Transpose(X_O);

	for (i = 0; i < id_body_EE.size(); i++) {
		/////	08. Compute End-effector position & rotation from MuJoCo !!
		for (j = 0; j < DOF3; j++) {
			p_EE_mj[i](j) = uData->site_xpos[i * DOF3 + j];						//	End-effecor Position from MuJoCo
			for (k = 0; k < DOF3; k++) {
				R_EE_mj[i](j, k) = uData->site_xmat[i * 9 + j * DOF3 + k];		//	End-effector Rotation from MuJoCo
			}
		}

		///// 09. End-Effector Jacobian & end-effector velocity from MuJoCo
		temp_vec3[0] = p_EE_mj[i](0);
		temp_vec3[1] = p_EE_mj[i](1);
		temp_vec3[2] = p_EE_mj[i](2);

		//	Linear & angular end-effector Jacobian matrix
		mj_jac(uModel, uData, jacp, jacr, temp_vec3, id_body_EE[i] + 1);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_EE_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_EE_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_EE_mj[i] = Jp_EE_mj[i] * Eigen::Transpose(X_O);
		Jr_EE_mj[i] = Jr_EE_mj[i] * Eigen::Transpose(X_O);

		/////	10. Time derivative of end-effector Jacobian
		Jdotp_EE_mj[i] = (Jp_EE_mj[i] - Pre_Jp_EE_mj[i]) / dT;
		Jdotr_EE_mj[i] = (Jr_EE_mj[i] - Pre_Jr_EE_mj[i]) / dT;

		Pre_Jp_EE_mj[i] = Jp_EE_mj[i];
		Pre_Jr_EE_mj[i] = Jr_EE_mj[i];
	}

    if (count_ctrl % int(3.0 / dT) == 0) {
        std::cout << YELLOW << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << RESET << std::endl << std::endl;

		{
			///	04-1. CoM pose of the system
			error = 0;
			p_G = p_G / bruce.mass_;
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_CoM_mj(j) - bruce.p_CoM_(j)) > error_precision ? 1 : 0));
				error = max(error, (abs(bruce.p_CoM_(j) - p_G(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 04-1. CoM position error (p_CoM_mj - p_CoM)" << endl;
				cout << (p_CoM_mj - bruce.p_CoM_) << endl;
			}

			///	04-2. CoM Jacobian of system
			error = 0;
			J_G = J_G / bruce.mass_;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(J_G_mj(j, k) - bruce.J_CoM_(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(J_G_mj(j, k) - J_G(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-2. CoM Jacobian error (J_G_mj - J_CoM)" << endl;
				cout << (J_G_mj - bruce.J_CoM_) << endl;
			}

			// ///	04-3. Time derivative of CoM Jacobian of system
			// error = 0;
			// Jdot_G = Jdot_G / m_G;
			// for (j = 0; j < DOF3; j++) {
			// 	for (k = 0; k < TOTAL_DOF; k++) {
			// 		error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
			// 	}
			// }
			// if (error == 1) {
			// 	cout << "# 04-3 : Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
			// 	cout << (Jdot_G - robot.Jdot_CoM) << endl;
			// }

			// error = 0;
			// for (j = 0; j < DOF3; j++) {
			// 	for (k = 0; k < TOTAL_DOF; k++) {
			// 		error = max(error, (abs(robot.C_p(j, k) - m_G * robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
			// 		error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
			// 	}
			// }
			// if (error == 1) {
			// 	cout << "# 04-3. Time derivative CoM Jacobian error (C_p -  m_G * Jdot_CoM)" << endl;
			// 	cout << (robot.C_p - m_G * robot.Jdot_CoM) << endl;
			// 	cout << "# 04-3. Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
			// 	cout << (Jdot_G - robot.Jdot_CoM) << endl;
			// }
		}

		//////////	05 : End-effector Kinematics expressed in {I}
		for (i = 0; i < id_body_EE.size(); i++) {
			///	05-1. Global end-effector position & rotation
			error = 0;
			// robot.getBodyPose(id_body_EE[i], p0_lnk2EE[i], R0_lnk2EE[i], p_EE[i], R_EE[i]);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_EE_mj[i](j) - p_EE[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_EE_mj[i](j, k) - R_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				std::cout << "# 05-1. " << i << "-th End-effector : Position error (p_EE_mj - p_EE)" << endl;
				std::cout << (p_EE_mj[i] - p_EE[i]) << endl;
				std::cout << "# 05-1. " << i << "-th End-effector : Rotation error (R_EE_mj - R_EE)" << endl;
				std::cout << (R_EE_mj[i] - R_EE[i]) << endl;
			}

			///	05-2. End-effector Jacobian expressed in {I}
			error = 0;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_EE_mj[i](j, k) - Jp_EE[i](j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jr_EE_mj[i](j, k) - Jr_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				std::cout << "# 05-2. " << i << "-th End-effector : Linear Jacobian error (Jp_EE_mj - Jp_EE)" << endl;
				std::cout << (Jp_EE_mj[i] - Jp_EE[i]) << endl;
				std::cout << "# 05-2. " << i << "-th End-effector : Angular Jacobian error (Jr_EE_mj - Jr_EE)" << endl;
				std::cout << (Jr_EE_mj[i] - Jr_EE[i]) << endl;
			}

			// ///	05-3. Check Time derivative of End-effector Jacobian : ��� �� �ٸ� �������, �� �� ��Ȯ�ϰ�~~~
			// error = 0;
			// robot.getBodyJacobDeriv(id_body_EE[i], Jdotp_EE[i], Jdotr_EE[i]);
			// for (j = 0; j < DOF3; j++) {
			// 	for (k = 0; k < TOTAL_DOF; k++) {
			// 		error = max(error, (abs(Jdotp_EE_mj[i](j, k) - Jdotp_EE[i](j, k)) > 0.01 ? 1 : 0));
			// 		error = max(error, (abs(Jdotr_EE_mj[i](j, k) - Jdotr_EE[i](j, k)) > 0.01 ? 1 : 0));
			// 	}
			// }
			// if (error == 1) {
			// 	std::cout << "# 05-3. " << i << "-th End-effector : Time derivative of linear Jacobian error (Jdotp_EE - Jdotp_EE_mj)" << endl;
			// 	std::cout << (Jdotp_EE[i] - Jdotp_EE_mj[i]) << endl;
			// 	std::cout << "# 05-3. " << i << "-th End-effector : Time derivative of angular Jacobian error (Jdotr_EE - Jdotr_EE_mj)" << endl;
			// 	std::cout << (Jdotr_EE[i] - Jdotr_EE_mj[i]) << endl;
			// }
		}


    }
}