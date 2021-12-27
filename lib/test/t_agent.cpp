
#include <gtest/gtest.h>
#include "agent.h"
#include "environment.h"


TEST(Agent, Ctor)
{
    auto a = Agent::createAgent(5);
    ASSERT_EQ(a->id(), 5);
}

TEST(Agent, Type)
{
    auto a = Agent::createAgent(5);
    ASSERT_EQ(a->type(), AgentType::EAgent);
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

TEST(Agent, SetEnvironment)
{
    auto a = Agent::createAgent(5);

    ASSERT_FALSE(a->hasEnvironment());

    auto e = Environment::createEnvironment(9);
    a->setEnvironment(e);

    ASSERT_TRUE(a->hasEnvironment());
}

TEST(Agent, MoveStandardImpl)
{
    auto a = Agent::createAgent(5);
    auto e = Environment::createEnvironment(9);
    a->setEnvironment(e);

    // nothing happens
    a->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    a->setVelocity(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->update(1.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    a->setAcceleration(Eigen::Vector2d(1.0, 2.0));
    a->setVelocity(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->update(0.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    // acceleration
    a->setAcceleration(Eigen::Vector2d(1.0, 2.0));
    a->setVelocity(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->update(1.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(1.0, 2.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(0.5, 1.0)).isMuchSmallerThan(0.0001));
    a->update(1.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001));

    // negative acceleration
    a->setAcceleration(Eigen::Vector2d(-1.0, -2.0));
    a->update(2.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
}


class CircEnv: public Environment
{
public:
    CircEnv(unsigned int id): Environment(id) {}
    virtual ~CircEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        // Circular environmet with radius 10. If move not possible, return
        // previous position.
        double radius = 10.0;
        if(destination.norm() < radius)
            return {true, destination};
        else
            return {false, origin};
    }

};

TEST(Agent, MoveSpecialEnv)
{
    auto a = Agent::createAgent(5);
    auto e = std::shared_ptr<CircEnv>(new CircEnv(3));
    a->setEnvironment(e);

    a->setVelocity(Eigen::Vector2d(10.0, 0.0));
    a->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    a->setPosition(Eigen::Vector2d(0.0, 0.0));

    // ok
    a->update(0.9);
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(9.0, 0.0)).isMuchSmallerThan(0.0001));
    a->update(0.05);
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(9.5, 0.0)).isMuchSmallerThan(0.0001));

    // not ok anymore -> position not changes
    a->update(0.1);
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(9.5, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Agent, AddGetSubAgents)
{
    auto a = Agent::createAgent(1);
    auto ac1 = Agent::createAgent(2);
    auto ac2 = Agent::createAgent(3);

    ac1->addSubAgent(Agent::createAgent(4));
    a->addSubAgent(ac1);
    a->addSubAgent(ac2);

    ASSERT_EQ(a->id(), 1);

    ASSERT_EQ(a->getSubAgents().size(), 2);
    ASSERT_EQ(a->getSubAgents().front()->id(), 2);
    ASSERT_EQ(a->getSubAgents().back()->id(), 3);

    ASSERT_EQ(a->getSubAgents().front()->getSubAgents().size(), 1);
    ASSERT_EQ(a->getSubAgents().front()->getSubAgents().front()->id(), 4);

    ASSERT_EQ(a->getSubAgents().back()->getSubAgents().size(), 0);
}

TEST(Agent, GetAllSubAgents)
{
    auto a = Agent::createAgent(1);
    auto ac1 = Agent::createAgent(2);
    auto ac2 = Agent::createAgent(3);

    ac1->addSubAgent(Agent::createAgent(4));
    a->addSubAgent(ac1);
    a->addSubAgent(ac2);

    auto l = a->getAllSubAgents();

    ASSERT_EQ(l.size(), 3);
}

TEST(Agent, UpdateAllAgents)
{
    auto e = Environment::createEnvironment(9);

    auto a = Agent::createAgent(1);
    auto ac1 = Agent::createAgent(2);
    auto ac1c1 = Agent::createAgent(4);
    auto ac2 = Agent::createAgent(3);

    ac1->addSubAgent(ac1c1);
    a->addSubAgent(ac1);
    a->addSubAgent(ac2);

    auto l = a->getAllSubAgents();
    l.push_back(a);

    // init all agents with same position and same speed
    std::for_each(l.begin(), l.end(), [e](auto q){
        q->setPosition(Eigen::Vector2d(0.0, 0.0));
        q->setVelocity(Eigen::Vector2d(2.0, 0.0));
        q->setEnvironment(e);
    });

    a->update(2.0);

    std::for_each(l.begin(), l.end(), [e](auto q){
        ASSERT_TRUE((q->getPosition() - Eigen::Vector2d(4.0, 0.0)).isMuchSmallerThan(0.0001));
        ASSERT_TRUE((q->getVelocity() - Eigen::Vector2d(2.0, 0.0)).isMuchSmallerThan(0.0001));
    });

}

TEST(Agent, MaxSpeedAccelration)
{
    auto env = Environment::createEnvironment(3);
    auto h = Agent::createAgent(11);
    h->setVelocityLimit(10.0);
    h->setAccelreationLimit(1.0);
    h->setEnvironment(env);

    // check set
    h->setVelocity(Eigen::Vector2d(0.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    h->setVelocity(Eigen::Vector2d(0.5, 0.6));
    h->setAcceleration(Eigen::Vector2d(0.1, 0.2));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(0.5, 0.6)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.1, 0.2)).isMuchSmallerThan(0.0001));

    h->setVelocity(Eigen::Vector2d(11.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(0.0, 2.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.0, 1.0)).isMuchSmallerThan(0.0001));

    // check velocity does not go over limit
    h->setVelocity(Eigen::Vector2d(0.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(1.0, 0.0));

    h->update(5.0);

    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    h->update(5.0);

    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));

    h->update(0.1);

    // max speed reached -> limit to 10
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Agent, MaxAcceleration)
{
    auto a = Agent::createAgent(5);
    a->setAccelreationLimit(10.0);
    a->setMaxAccelerationInDirection(Eigen::Vector2d(1.0, 0.0));
    ASSERT_TRUE((a->getAcceleration() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Agent, MaxVelocity)
{
    auto a = Agent::createAgent(5);
    a->setVelocityLimit(15.0);
    a->setMaxVelocityInDirection(Eigen::Vector2d(0.0, 1.0));
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 15.0)).isMuchSmallerThan(0.0001));
}
