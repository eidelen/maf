
#include <gtest/gtest.h>
#include "objective.h"

TEST(Objective, ConstructDefault)
{
    auto a1 = Agent::createAgent(4);
    auto o = std::shared_ptr<Objective>(new Objective(2, a1));
    ASSERT_EQ(o->id(), 2);
}

class MySimpleObjective: public Objective
{
public:
    MySimpleObjective(unsigned int id, AgentWP agent) : Objective(id, agent)
    {
    }

    void react(double timeStep) override
    {
        // set the target position
        m_agent.lock()->setPosition(Eigen::Vector2d(2.0, 4.0));

    }

    bool isDone() const override
    {
        return (m_agent.lock()->getPosition() - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001);
    }
};

TEST(Objective, SimpleObjective)
{
    auto a1 = Agent::createAgent(4);
    auto o = std::shared_ptr<MySimpleObjective>(new MySimpleObjective(2, a1));

    ASSERT_EQ(o->id(), 2);

    ASSERT_FALSE(o->isDone());

    o->react(1.0);

    ASSERT_TRUE(o->isDone());
}
