#include "WBC/wbc_base.hpp"

wbcBase::wbcBase()
{
    std::cout << "wbcBase Constructor" << std::endl;
    numDecisionVars_ = 10;
}

Eigen::VectorXd wbcBase::update(Robot robot)
{
    std::cout << "B" << std::endl;
    // robot_ = robot;
    // std::cout << "wbcBase::update()" << std::endl;
    return Eigen::VectorXd::Zero(5);
}

Task wbcBase::formulateFloatingBaseConstraint()
{
    Task a;
    // Mqddot + Cqdot + G = Sf * f
    return a;
}

Task wbcBase::formulateFrictionConeConstraint()
{
    Task a;
    return a;
}