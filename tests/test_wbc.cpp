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

    // for(int i=0; i<2; i++) {
    //     wbc_.update(bruce_, controller_);
    // }
}

TEST_F(TestWBC, testWBCbase_shared_ptr)
{
    BRUCEController controller;
    Robot bruce;
    std::shared_ptr<wbcBase> wbc;
    wbc = std::make_shared<WeightedWBC>();

    // for(int i=0; i<2; i++) {
    //     wbc->update(bruce, controller);
    // }
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

    Task taskA;
    taskA.a_ = Eigen::MatrixXd::Random(3, 16);
    taskA.b_ = Eigen::MatrixXd::Random(3, 1);
    taskA.d_ = Eigen::MatrixXd::Random(6, 16);
    taskA.f_ = Eigen::MatrixXd::Random(6, 1);
    Task taskB;
    taskB.a_ = Eigen::MatrixXd::Random(6, 16);
    taskB.b_ = Eigen::MatrixXd::Random(6, 1);
    taskB.d_ = Eigen::MatrixXd::Random(6, 16);
    taskB.f_ = Eigen::MatrixXd::Random(6, 1);

    Task taskRes = taskA + taskB;

    // std::cout << "a: " << std::endl << taskRes.a_ << std::endl;
    // std::cout << "b: " << std::endl << taskRes.b_.transpose() << std::endl;
    // std::cout << "d: " << std::endl << taskRes.d_ << std::endl;
    // std::cout << "f: " << std::endl << taskRes.f_.transpose() << std::endl;

    Eigen::MatrixXd Q1 = taskRes.a_.transpose() * taskRes.a_;
    Eigen::MatrixXd Q2;

    Q2 = taskA.a_.transpose() * taskA.a_ + taskB.a_.transpose() * taskB.a_;

    // std::cout << Q1 << std::endl;
    // std::cout << Q2 << std::endl;

    // EXPECT_EQ(Q1, Q2);
    EXPECT_TRUE((Q1-Q2).norm() < 1e-10);
}