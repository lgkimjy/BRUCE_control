#include <iostream>

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "WBC/wbc_base.hpp"

class WeightedWBC : public wbcBase
{
private:
public:
    WeightedWBC() {};
    ~WeightedWBC() {};

    Eigen::VectorXd update();
protected:
};