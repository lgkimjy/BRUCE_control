#include "gtest/gtest.h"
#include <Eigen/Dense>

class TestEigen : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }
    virtual void TearDown()
    {

    }
};

TEST_F(TestEigen, JcfcTest)
{
    // Augmented_Jc.transpose() * Aug_fc or \sum_i Jc[i].transpose() * fc[i]
    double tol = 1e-10;
    std::vector<Eigen::Matrix<double, 3, 10>>   Jc(4);
    std::vector<Eigen::Matrix<double, 3, 1>>    fc(4);

    for(int i=1; i<=Jc.size(); i++) {
        Jc[i-1].setRandom();
        fc[i-1] = i * fc[i-1].setOnes();
    }

    Eigen::Matrix<double, 12, 10>   Aug_Jc;
    Eigen::Matrix<double, 12, 1>    Aug_fc;
    Aug_Jc.setZero();
    for(int i=0; i<Jc.size(); i++) {
        Aug_Jc.block(3*i, 0, 3, 10) = Jc[i];
        Aug_fc.block(3*i, 0, 3, 1) = fc[i];
    }

    Eigen::Matrix<double, 10, 1>     res;
    res.setZero();
    for(int i=0; i<Jc.size(); i++) {
        res += Jc[i].transpose() * fc[i];
    }

    // std::cout << Aug_Jc.transpose() * Aug_fc << std::endl;
    // std::cout << res << std::endl;
    // EXPECT_EQ(Aug_Jc.transpose() * Aug_fc, res);
    EXPECT_TRUE((Aug_Jc.transpose() * Aug_fc - res).norm() < tol);
}