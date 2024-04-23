#include "gtest/gtest.h"

#include <Eigen/Dense>
// #include "WBC/wbc_base.hpp"
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
    Robot bruce;
    std::shared_ptr<wbcBase> wbc;
    wbc = std::make_shared<WeightedWBC>();

    WeightedWBC wbc_;


    for(int i=0; i<10; i++)
    {
        // wbc->update(bruce);
        wbc_.updates(bruce);
    }
}