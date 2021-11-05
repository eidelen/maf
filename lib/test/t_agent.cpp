
#include <gtest/gtest.h>
#include "agent.h"


TEST(Agent, Ctor)
{
    auto a = Agent::createAgent(5);
    ASSERT_EQ(a->id(), 5);
}

TEST(Agent, Radius)
{
    auto a = Agent::createAgent(5);
    a->setRadius(2.55);
    ASSERT_NEAR(a->getRadius(), 2.55, 0.0000001);
}

TEST(Agent, Position)
{
    auto a = Agent::createAgent(5);
    a->setPosition(Eigen::Vector2d(2.5, 4.1));
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(2.5, 4.1)).isMuchSmallerThan(0.0001));
}

TEST(Agent, Volocity)
{
    auto a = Agent::createAgent(5);
    a->setVelocity(Eigen::Vector2d(2.5, 6.1));
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(2.5, 6.1)).isMuchSmallerThan(0.0001));
}


