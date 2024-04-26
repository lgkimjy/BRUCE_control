#pragma once

#include "WBC/wbc_base.hpp"

#include <qpOASES.hpp>

class BRUCEController; // Forward declaration

class WBIC : public wbcBase
{
private:

public:
    // using wbcBase::wbcBase;
    WBIC() {std::cout<<"WBIC Constructor"<<std::endl;}; 
    ~WBIC() {std::cout<<"WBIC Destructor"<<std::endl;};

    Eigen::VectorXd update(Robot &robot, BRUCEController& controller, Eigen::VectorXd qcmd, Eigen::VectorXd rho);

protected:
    virtual Task formulateConstraints(Eigen::VectorXd qcmd, Eigen::VectorXd rho);
    virtual Task formulateRelaxationTask();
};