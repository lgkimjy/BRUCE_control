#include "gtest/gtest.h"

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

TEST_F(TestPino, test1)
{
    EXPECT_EQ(1, 1);
}