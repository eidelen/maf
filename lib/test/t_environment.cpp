
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

TEST(Environment, AgentDistance)
{
    auto a = Agent::createAgent(4);
    auto b = Agent::createAgent(5);

    a->setPosition(Eigen::Vector2d(-4.0, 0.0));
    b->setPosition(Eigen::Vector2d(0.0, 3.0));

    Eigen::Vector2d res = Environment::computeDistance(a, b);

    ASSERT_TRUE((res - Eigen::Vector2d(4.0,3.0)).isMuchSmallerThan(0.0001));
}

