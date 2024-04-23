#include "WBC/wbc_base.hpp"

#include <qpOASES.hpp>

class WBIC : public wbcBase
{
private:
    double swingLegWeight_;
    double baseAccelWeight_;
    double contactForceWeight_;

public:
    using wbcBase::wbcBase;
    WBIC();
    ~WBIC() {};

    Eigen::VectorXd update(Eigen::VectorXd qcmd, Eigen::VectorXd rho);

protected:
    virtual Task formulateConstraints();
    virtual Task formulateWeightedTask();
};