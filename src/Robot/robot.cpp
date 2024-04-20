/**
 * @file: robot.cpp
 * @brief: Robot KinDyn Computation Using Pinocchio
 * @author: Jun Young Kim
 */

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
    // Ref: https://github.com/stack-of-tasks/pinocchio/issues/735
    std::string urdf_filename = std::string(CMAKE_SOURCE_DIR"/model/bruce/urdf/bruce.urdf");
    pinocchio::JointModelFreeFlyer root_joint;                          // for humanoid robots, floating base coordinates
    pinocchio::urdf::buildModel(urdf_filename, root_joint, model_);     // build the model from the URDF file (floating base)
    std::cout << "Model name: " << GREEN << model_.name << RESET << std::endl;

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
    pinocchio::forwardKinematics(model_, data_, xi_quat_, xidot_);
    pinocchio::computeJointJacobians(model_, data_, xi_quat_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::crba(model_, data_, xi_quat_);           // Computes the upper triangular part of the joint space inertia matrix M by using the Composite Rigid Body Algorithm
    // Print out the placement of each joint of the kinematic tree
    // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    //     std::cout << std::setw(24) << std::left
    //             << model_.names[joint_id] << ": "
    //             << std::fixed << std::setprecision(2)
    //             << data_.oMi[joint_id].translation().transpose() // Vector of absolute joint placements (wrt the world). SE3
    //             << std::endl;
    // std::cout << std::endl;    
}

void Robot::computeDynamics()
{
    pinocchio::nonLinearEffects(model_, data_, xi_quat_, xidot_);
    pinocchio::computeCoriolisMatrix(model_, data_, xi_quat_, xidot_);
    pinocchio::computeJointJacobiansTimeVariation(model_, data_, xi_quat_, xidot_);
    pinocchio::computeGeneralizedGravity(model_, data_, xi_quat_);
    pinocchio::rnea(model_, data_, xi_quat_, xidot_, xiddot_);
    // pinocchio::computeCentroidalDynamics(model_, data_, xi_quat_, xidot_);

    // compute Centroidal Dynamics

}

void Robot::computeCoMMotion()
{
    pinocchio::centerOfMass(model_, data_, xi_quat_);
    pinocchio::jacobianCenterOfMass(model_, data_, xi_quat_);
    p_CoM_ = data_.com[0];
    J_CoM_ = data_.Jcom;
    pddot_CoM_ = J_CoM_ * xidot_;
}

// Ref: https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-dynamics.cpp
Eigen::VectorXd Robot::computeInverseDynamics(Eigen::VectorXd xi_quat_cmd, Eigen::VectorXd xidot_cmd, Eigen::VectorXd xiddot_cmd)
{
    torq_ = pinocchio::rnea(model_, data_, xi_quat_cmd, xidot_cmd, xiddot_cmd);
    return torq_;
}