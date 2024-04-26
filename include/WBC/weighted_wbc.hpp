#pragma once

#include "WBC/wbc_base.hpp"

#include "Robot/robot.hpp"
#include <qpOASES.hpp>

class BRUCEController; // Forward declaration

class WeightedWBC : public wbcBase
{
public:
    // using wbcBase::wbcBase;
    WeightedWBC() {std::cout<<"WeightedWBC Constructor"<<std::endl;}; 
    ~WeightedWBC() {std::cout<<"WeightedWBC Destructor"<<std::endl;};

    Eigen::VectorXd update(Robot &robot, BRUCEController &controller);

protected:
    virtual Task formulateConstraints();
    virtual Task formulateWeightedTask();

private:
    double swingLegWeight_;
    double baseAccelWeight_;
    double contactForceWeight_;
};