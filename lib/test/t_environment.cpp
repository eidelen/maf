
#include <gtest/gtest.h>
#include "environment.h"

TEST(Environment, Id)
{
    auto e = Environment::createEnvironment(9);
    ASSERT_EQ(e->id(), 9);
}

TEST(Environment, MovePossibleBaseImpl)
{
    auto e = Environment::createEnvironment(9);
    auto[possible, newPos] = e->possibleMove(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 9.0));
    ASSERT_TRUE(possible);
    ASSERT_TRUE((newPos - Eigen::Vector2d(5.0, 9.0)).isMuchSmallerThan(0.0001));
}
