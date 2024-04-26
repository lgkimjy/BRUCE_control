#include "gtest/gtest.h"

#include <Eigen/Dense>

#include "bruce_controller.hpp"
#include "WBC/weighted_wbc.hpp"

class TestWBC : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }
    virtual void TearDown()
    {

    }
};

TEST_F(TestWBC, testWBCbase)
{
    BRUCEController controller_;
    Robot bruce_;
    WeightedWBC wbc_;

    for(int i=0; i<2; i++) {
        wbc_.update(bruce_, controller_);
    }
}

TEST_F(TestWBC, testWBCbase_shared_ptr)
{
    BRUCEController controller;
    Robot bruce;
    std::shared_ptr<wbcBase> wbc;
    wbc = std::make_shared<WeightedWBC>();

    for(int i=0; i<2; i++) {
        wbc->update(bruce, controller);
    }
}

TEST_F(TestWBC, test_EigenConcatenate)
{
    Eigen::MatrixXd data = Eigen::MatrixXd::Ones(5, 6);
    Eigen::MatrixXd j = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd s = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd mat = (Eigen::MatrixXd(5, 16) << data, j, s).finished();

    // std::cout << mat << std::endl;
    // mat.noalias();
    // std::cout << mat << std::endl;

    EXPECT_EQ(mat.rows(), 5);
    EXPECT_EQ(mat.cols(), 16);
}