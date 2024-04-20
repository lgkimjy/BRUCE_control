#include <Eigen/Dense>
#include <iostream>

class wbcBase
{
private:
public:
    wbcBase(){};
    ~wbcBase(){};

    virtual Eigen::VectorXd update();
protected:
    void formulateContactTask();
    void formulateCoMTask();
    void formulateOrientationTask();
    void formulateEETask();
    void formulateJointTask();
};