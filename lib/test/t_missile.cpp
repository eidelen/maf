
#include <gtest/gtest.h>
#include "missile.h"
#include "environment.h"

TEST(Missile, TypeAndId)
{
    auto m = std::shared_ptr<Missile>(new Missile(4));
    ASSERT_EQ(m->type(), AgentType::EMissile);
    ASSERT_EQ(m->id(), 4);
}

TEST(Missile, BasicStateAndTarget)
{
    auto m = std::shared_ptr<Missile>(new Missile(4));
    ASSERT_EQ(m->status(), Missile::Idle);

    m->fire(99);

    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_EQ(m->target(), 99);
}

TEST(Missile, FollowAndDetonate)
{
    auto e = Environment::createEnvironment(0);

    // slow missile
    auto m = std::shared_ptr<Missile>(new Missile(1));
    m->setPosition(Eigen::Vector2d(0.0, 0.0));
    m->setMaxSpeed(5.0);
    m->setMaxAccelreation(1.0);
    m->setEnvironment(e);

    // target 100m away
    auto t = Agent::createAgent(2);
    t->setPosition(Eigen::Vector2d(100.0, 0.0));
    t->setEnvironment(e);

    e->addAgent(m);
    e->addAgent(t);

    e->update(1.0);

    // idle - no movement
    ASSERT_EQ(m->status(), Missile::Idle);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    // fire
    m->fire(2);
    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(0.5, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));

    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(2.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(2.0, 0.0)).isMuchSmallerThan(0.0001));

    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(4.5, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(3.0, 0.0)).isMuchSmallerThan(0.0001));

    e->update(2.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(12.5, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    // max speed reached

    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(17.5, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getAcceleration() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));
}
