#include "gtest/gtest.h"

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>

class TestPino : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }
    virtual void TearDown()
    {

    }
};

TEST_F(TestPino, testActInv)
{
    Eigen::Vector3d v = {1, 2, 3};
    Eigen::Matrix3d R = Eigen::AngleAxisd(1.0471975512, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pinocchio::SE3 A(R, v);
    std::cout << A << std::endl;

    Eigen::Vector3d mulipler = {1, 1, 1};
    std::cout << A.actInv(mulipler).transpose() << std::endl;
    std::cout << A.act(mulipler).transpose() << std::endl;

    EXPECT_EQ(A.rotation().transpose() * mulipler, (R.transpose() * mulipler));
}