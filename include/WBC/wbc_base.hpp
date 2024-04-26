#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Robot/robot.hpp"
#include "WBC/Task.hpp"
// #include "Contact"

class BRUCEController; // Forward declaration

class wbcBase
{
private:
public:
    wbcBase();
    ~wbcBase(){std::cout<<"WBC Base Destructor"<<std::endl;};

    Robot robot_;
    double numDecisionVars_;

    virtual Eigen::VectorXd update(Robot &robot, BRUCEController &controller);
    
    // Varaibale related to formulate Tasks and Constraints
    Eigen::MatrixXd             Sf_;
    Eigen::VectorXd	            p_EE_;          // Augmented End-effector Jacobain
    Eigen::MatrixXd             Jp_contact_;    // Augmented Linear Contact Jacobian 
    Eigen::MatrixXd             Jdotp_contact_;

protected:
    Task formulateContactTask();
    Task formulateMomentumTask();
    Task formulateTorqueLimitsTask();
    Task formulateSwingLegTask();

    Task formulateFloatingBaseConstraint();
    Task formulateFrictionConeConstraint();
    Task formulateContactConstraint();

    size_t getNumDecisionVars() const { return numDecisionVars_; }
};