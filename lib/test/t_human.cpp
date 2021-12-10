
#include <gtest/gtest.h>
#include "human.h"
#include "environment.h"

TEST(Human, Type)
{
    auto a = Human::createHuman(11, 10.0, 1.0, 3.0, -1.0);
    ASSERT_EQ(a->type(), AgentType::EHuman);
}

TEST(Human, SlowDown)
{
    auto env = Environment::createEnvironment(3);
    auto h = Human::createHuman(11, 10.0, 4.0, 3.0);
    h->setEnvironment(env);

    h->setVelocity(Eigen::Vector2d(10.0, 0.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));

    // human gets slower, as no other agent around
    double lastSpeed = h->getVelocity().norm();

    h->update(1.0);
    ASSERT_GT(lastSpeed, h->getVelocity().norm());
    lastSpeed = h->getVelocity().norm();

    h->update(1.0);
    ASSERT_GT(lastSpeed, h->getVelocity().norm());
    lastSpeed = h->getVelocity().norm();

    h->update(1.0);
    ASSERT_GT(lastSpeed, h->getVelocity().norm());
}

TEST(Human, LongSlowDown)
{
    auto env = Environment::createEnvironment(3);
    auto h = Human::createHuman(11, 10.0, 4.0, 3.0, 0.0001);
    h->setEnvironment(env);

    h->setVelocity(Eigen::Vector2d(10.0, 0.0));
    double lastSpeed = h->getVelocity().norm();

    for(size_t k = 0; k < 2000; k++)
    {
        h->update(0.01);
        double cSpeed = h->getVelocity().norm();
        ASSERT_TRUE(cSpeed < lastSpeed || cSpeed < 10e-5);
        lastSpeed = cSpeed;
    }
}

TEST(Human, MoveAwayFromOtherHuman)
{
    auto env = Environment::createEnvironment(3);

    auto h1 = Human::createHuman(0, 10.0, 10.0, 10.0, 0.0001);
    h1->setEnvironment(env);
    h1->setVelocity(Eigen::Vector2d(0.0, 0.0));

    // Dummy does not care
    auto dummy = Human::createHuman(1, 0.0, 0.0, 0.0);
    dummy->disableReacting(true);
    dummy->setEnvironment(env);
    dummy->setVelocity(Eigen::Vector2d(0.0, 0.0));

    env->addAgent(h1);
    env->addAgent(dummy);

    // Dont care about a second agent outside the obsDistance
    h1->setPosition(Eigen::Vector2d(0.0, 0.0));
    dummy->setPosition(Eigen::Vector2d(-20.0, 0.0));
    env->update(1.0);
    ASSERT_TRUE((h1->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((dummy->getPosition() - Eigen::Vector2d(-20.0, 0.0)).isMuchSmallerThan(0.0001));

    // h1 moves away from dummy in x-direction
    dummy->setPosition(Eigen::Vector2d(-1.0, 0.0));
    double lastDist = 1.0;

    while(lastDist < 10.0) // After 10m h1 does not care and slows down.
    {
        env->update(0.1);

        // dummy stays
        ASSERT_TRUE((dummy->getPosition() - Eigen::Vector2d(-1.0, 0.0)).isMuchSmallerThan(0.0001));
        double dist = (dummy->getPosition().transpose() - h1->getPosition().transpose()).norm();

        ASSERT_GT(dist, lastDist);
        lastDist = dist;
    }
}
