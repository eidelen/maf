
#include <gtest/gtest.h>
#include "target.h"
#include "environment.h"

TEST(Target, TypeAndIdAndPosAndRange)
{
    auto t = Target::createTarget(33, Eigen::Vector2d(3.0, 4.0), 20.0);
    ASSERT_EQ(t->type(), AgentType::ETarget);
    ASSERT_EQ(t->id(), 33);
    ASSERT_TRUE((t->getPosition() - Eigen::Vector2d(3.0, 4.0)).isMuchSmallerThan(0.0001));
    ASSERT_NEAR(t->range(), 20.0, 0.0001);
}

TEST(Target, BasicWithinRange)
{
    auto t = Target::createTarget(1, Eigen::Vector2d(0.0, 0.0), 1.0);
    auto a1 = Agent::createAgent(2);
    auto a2 = Agent::createAgent(3);
    auto e = Environment::createEnvironment(0);
    std::vector<std::shared_ptr<Agent>> al = {t, a1, a2};
    std::for_each(al.begin(), al.end(), [&e](std::shared_ptr<Agent> z){
        z->setEnvironment(e);
        z->setVelocity(Eigen::Vector2d(0.0, 0.0));
        z->setAcceleration(Eigen::Vector2d(0.0, 0.0));
        e->addAgent(z);
    });


    // no agent in range
    a1->setPosition(Eigen::Vector2d(-2.0, 0.0));
    a2->setPosition(Eigen::Vector2d(0.0, 2.0));
    e->update(1.0);
    auto r1 = t->getAgentsInSensorRange();
    ASSERT_EQ(0, r1.size());

    // a2, a1 agent in range
    a1->setPosition(Eigen::Vector2d(0.0, 0.5));
    a2->setPosition(Eigen::Vector2d(-0.5, 0.5));
    e->update(1.0);
    auto r3 = t->getAgentsInSensorRange();
    ASSERT_EQ(2, r3.size());
}
