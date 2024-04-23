#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "Robot/robot.hpp"
#include "WBC/Task.hpp"

class wbcBase
{
private:
public:
    wbcBase();
    ~wbcBase(){std::cout<<"WBC Base Destructor"<<std::endl;};

    size_t numDecisionVars_;

    Eigen::VectorXd update(Robot robot);
protected:
    void formulateContactTask();
    void formulateCoMTask();
    void formulateOrientationTask();
    void formulateEETask();
    void formulateJointTask();

    Task formulateFloatingBaseConstraint();
    Task formulateFrictionConeConstraint();

    size_t getNumDecisionVars() const { return numDecisionVars_; }
};