#include "Robot/robot.hpp"

Robot::Robot() 
{
    std::cout << "Robot Constructor" << std::endl;
}

Robot::~Robot() 
{
    std::cout << "Robot Destructor" << std::endl;
}


void Robot::loadRobotModel()
{
    std::string urdf_filename = std::string(CMAKE_SOURCE_DIR"/model/bruce/urdf/bruce.urdf");
    pinocchio::JointModelFreeFlyer root_joint;                          // for humanoid robots, floating base coordinates
    pinocchio::urdf::buildModel(urdf_filename, root_joint, model_);     // option1: build the model from the URDF file (floating base)
    // pinocchio::urdf::buildModel(urdf_filename, model_);              // option2: build the model from the URDF file (fixed base)
    std::cout << model_.name << std::endl;

    data_ = pinocchio::Data(model_);
    
    nv_ = model_.nv;
    nq_ = model_.nq;

    jnames_.clear();
    int names_size = model_.names.size();
    jnames_.reserve(names_size);

    for (int i = 0; i < names_size; i++)
    {
        const std::string jname = model_.names[i];
        std::cout << "Joint Id: " << model_.getJointId(jname) << " # " << model_.idx_qs[model_.getJointId(jname)] << " " << model_.idx_vs[model_.getJointId(jname)] << " Name: " << jname << std::endl;
        //Do not insert "universe" joint
        if (jname.compare("universe") != 0 && jname.compare("root_joint") != 0)
        {
            jnames_.push_back(jname);
        }
    }
}

// Ref: https://github.com/stack-of-tasks/pinocchio/blob/master/examples/overview-urdf.cpp
void Robot::computeForwardKinematics() 
{
    // Create data required by the algorithms
    pinocchio::forwardKinematics(model_, data_, xi_quat, xidot);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, xi_quat);
    // Print out the placement of each joint of the kinematic tree
    // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    //     std::cout << std::setw(24) << std::left
    //             << model_.names[joint_id] << ": "
    //             << std::fixed << std::setprecision(2)
    //             << data_.oMi[joint_id].translation().transpose() // Vector of absolute joint placements (wrt the world). SE3
    //             << std::endl;
    // std::cout << std::endl;    
}

// Ref: https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-dynamics.cpp
void Robot::computeInverseDynamics()
{
    // pinocchio::Data data(model);

    // Sample a random joint configuration, joint velocities and accelerations
    // Eigen::VectorXd q = randomConfiguration(model);       // in rad for the UR5
    // Eigen::VectorXd v = Eigen::VectorXd::Zero();  // in rad/s for the UR5
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);  // in rad/s² for the UR5

    // Eigen::VectorXd tau = pinocchio::rnea(model_, data_, xi_quat, xidot, a);
    // std::cout << tau.transpose() << std::endl;
}