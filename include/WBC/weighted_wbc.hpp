#include "WBC/wbc_base.hpp"

#include "Robot/robot.hpp"
#include <qpOASES.hpp>

class WeightedWBC : public wbcBase
{
public:
    using wbcBase::wbcBase;
    // WeightedWBC() {}; 
    ~WeightedWBC() {std::cout<<"WeightedWBC Destructor"<<std::endl;};

    Eigen::VectorXd updates(Robot robot);

protected:
    virtual Task formulateConstraints();
    virtual Task formulateWeightedTask();

private:
    double swingLegWeight_;
    double baseAccelWeight_;
    double contactForceWeight_;
};