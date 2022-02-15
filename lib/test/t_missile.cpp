
#include <gtest/gtest.h>
#include <iostream>
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

TEST(Missile, ApproachAndDetonate)
{
    auto e = Environment::createEnvironment(0);

    // slow missile
    auto m = std::shared_ptr<Missile>(new Missile(1));
    m->setPosition(Eigen::Vector2d(0.0, 0.0));
    m->setVelocityLimit(5.0);
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

    // fire: missile is at full speed from very beginning -> acceleration does not matter
    m->fire(2);
    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    e->update(1.0);
    ASSERT_EQ(m->status(), Missile::Launched);
    ASSERT_TRUE((m->getPosition() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((m->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    // wait till explosion
    while(m->status() == Missile::Launched)
    {
        e->update(0.12);

        // fail when fly by without detonation
        ASSERT_LT(m->getPosition()(0), 102.0);
    }

    ASSERT_EQ(m->status(), Missile::Detonated);
}

TEST(Missile, TargetTowardsMissile)
{
    auto e = Environment::createEnvironment(0);

    // slow missile
    auto m = std::shared_ptr<Missile>(new Missile(1));
    m->setPosition(Eigen::Vector2d(0.0, 0.0));
    m->setVelocityLimit(1.0);
    m->setEnvironment(e);

    // target moving towards missile
    auto t = Agent::createAgent(2);
    t->setPosition(Eigen::Vector2d(10.0, 0.0));
    t->setVelocity(Eigen::Vector2d(-1.0, 0.0));
    t->setEnvironment(e);

    e->addAgent(m);
    e->addAgent(t);

    m->fire(2);

    e->update(0.1);

    ASSERT_TRUE(m->status() == Missile::Launched);

    for(int k = 0; k < 100; k++)
    {
        e->update(0.1);
    }

    // missile exploded in between
    ASSERT_EQ(m->status(), Missile::Detonated);
    ASSERT_GT(m->getPosition()(0), 0.0);
    ASSERT_LT(m->getPosition()(0), 10.0);
}

TEST(Missile, TargetAwayFromMissile)
{
    auto e = Environment::createEnvironment(0);

    // slow missile
    auto m = std::shared_ptr<Missile>(new Missile(1));
    m->setPosition(Eigen::Vector2d(0.0, 0.0));
    m->setVelocityLimit(2.0);
    m->setEnvironment(e);

    // target moving away from missile
    auto t = Agent::createAgent(2);
    t->setPosition(Eigen::Vector2d(10.0, 0.0));
    t->setVelocity(Eigen::Vector2d(1.0, 0.0));
    t->setEnvironment(e);

    e->addAgent(m);
    e->addAgent(t);

    m->fire(2);

    for(int k = 0; k < 120; k++)
    {
        e->update(0.1);
    }

    // missile exploded in between
    ASSERT_EQ(m->status(), Missile::Detonated);
    ASSERT_GT(m->getPosition()(0), 19.7);
    ASSERT_LT(m->getPosition()(0), 20.3);
}
