
#include <gtest/gtest.h>
#include "proximity_sensor.h"
#include "environment.h"

TEST(ProxSensor, TypeAndId)
{
    auto p = ProximitySensor::createProxSensor(33, 20.0);
    ASSERT_EQ(p->type(), AgentType::EProxSensor);
    ASSERT_EQ(p->id(), 33);
}

TEST(ProxSensor, SetGetRange)
{
    auto p = ProximitySensor::createProxSensor(33, 20.0);
    ASSERT_NEAR(p->range(), 20.0, 0.0001);

    p->setRange(31.9);
    ASSERT_NEAR(p->range(), 31.9, 0.0001);
}

TEST(ProxSensor, AgentsInRang)
{
    // prepare 1 sensor and two agents
    auto p = ProximitySensor::createProxSensor(1, 10.0);
    auto a1 = Agent::createAgent(2);
    auto a2 = Agent::createAgent(3);
    auto e = Environment::createEnvironment(0);
    std::vector<std::shared_ptr<Agent>> al = {p, a1, a2};
    std::for_each(al.begin(), al.end(), [&e](std::shared_ptr<Agent> z){
        z->setEnvironment(e);
        z->setVelocity(Eigen::Vector2d(0.0, 0.0));
        z->setAcceleration(Eigen::Vector2d(0.0, 0.0));
        e->addAgent(z);
    });


    // no agent in range (10m)
    p->setPosition(Eigen::Vector2d(0.0, 0.0));
    a1->setPosition(Eigen::Vector2d(11.0, 0.0));
    a2->setPosition(Eigen::Vector2d(0.0, 11.0));
    e->update(1.0);
    auto r1 = p->getAgentsInSensorRange();
    ASSERT_EQ(0, r1.size());

    // a1 agent in range (10m)
    p->setPosition(Eigen::Vector2d(0.0, 0.0));
    a1->setPosition(Eigen::Vector2d(0.0, -9.0));
    a2->setPosition(Eigen::Vector2d(0.0, 11.0));
    e->update(1.0);
    auto r2 = p->getAgentsInSensorRange();
    ASSERT_EQ(1, r2.size());
    ASSERT_EQ(2, r2.at(0).targetId);
    ASSERT_NEAR(9.0, r2.at(0).dist, 0.0001);

    // a2, a1 agent in range (10m)
    p->setPosition(Eigen::Vector2d(0.0, 0.0));
    a1->setPosition(Eigen::Vector2d(0.0, -8.0));
    a2->setPosition(Eigen::Vector2d(0.0, 5.0));
    e->update(1.0);
    auto r3 = p->getAgentsInSensorRange();
    ASSERT_EQ(2, r3.size());
    ASSERT_EQ(3, r3.at(0).targetId);
    ASSERT_NEAR(5.0, r3.at(0).dist, 0.0001);
    ASSERT_EQ(2, r3.at(1).targetId);
    ASSERT_NEAR(8.0, r3.at(1).dist, 0.0001);
}
