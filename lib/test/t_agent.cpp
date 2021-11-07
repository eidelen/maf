
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

TEST(Agent, Acceleration)
{
    auto a = Agent::createAgent(5);
    a->setAcceleration(Eigen::Vector2d(8.5, 6.1));
    ASSERT_TRUE((a->getAcceleration() - Eigen::Vector2d(8.5, 6.1)).isMuchSmallerThan(0.0001));
}

TEST(Agent, LinearMotion)
{
    auto a = Agent::createAgent(5);

    // no velocity, no acceleration -> no movement
    a->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    a->setVelocity(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    auto [p0, v0] = a->computeMotion(1.0);
    ASSERT_TRUE((p0 - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v0 - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    // constant speed, no acceleration
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    a->setVelocity(Eigen::Vector2d(2.0, 7.0));
    auto [p1, v1] = a->computeMotion(1.0);
    ASSERT_TRUE((p1 - Eigen::Vector2d(2.0, 7.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v1 - Eigen::Vector2d(2.0, 7.0)).isMuchSmallerThan(0.0001));
    auto [p2, v2] = a->computeMotion(10.0);
    ASSERT_TRUE((p2 - Eigen::Vector2d(20.0, 70.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v2 - Eigen::Vector2d(2.0, 7.0)).isMuchSmallerThan(0.0001));
}

TEST(Agent, AccelerationMotion)
{
    auto a = Agent::createAgent(5);

    a->setAcceleration(Eigen::Vector2d(2.0, 4.0));
    a->setVelocity(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    auto [p0, v0] = a->computeMotion(1.0);
    std::cout << "p0: " << p0.transpose() << std::endl;
    std::cout << "v1: " << v0.transpose() << std::endl;
    ASSERT_TRUE((p0 - Eigen::Vector2d(1.0, 2.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v0 - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001));
    auto [p1, v1] = a->computeMotion(2.0);
    ASSERT_TRUE((p1 - Eigen::Vector2d(4.0, 8.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v1 - Eigen::Vector2d(4.0, 8.0)).isMuchSmallerThan(0.0001));
    auto [p2, v2] = a->computeMotion(4.0);
    ASSERT_TRUE((p2 - Eigen::Vector2d(16.0, 32.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v2 - Eigen::Vector2d(8.0, 16.0)).isMuchSmallerThan(0.0001));

    a->setVelocity(Eigen::Vector2d(1.0, 2.0));
    auto [p3, v3] = a->computeMotion(1.0);
    ASSERT_TRUE((p3 - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((v3 - Eigen::Vector2d(3.0, 6.0)).isMuchSmallerThan(0.0001));
}

