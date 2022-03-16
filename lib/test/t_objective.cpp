
#include <gtest/gtest.h>
#include "objective.h"

TEST(Objective, ConstructDefault)
{
    auto o = std::shared_ptr<Objective>(new Objective(2));
    ASSERT_EQ(o->id(), 2);
}

class MySimpleObjective: public Objective
{
public:
    MySimpleObjective(unsigned int id) : Objective(id)
    {
    }

    void react(double timeStep, AgentSP a) override
    {
        // set the target position
        a->setPosition(Eigen::Vector2d(2.0, 4.0));

    }

    bool isDone(AgentSP a) const override
    {
        return (a->getPosition() - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001);
    }
};

TEST(Objective, SimpleObjective)
{
    auto a1 = Agent::createAgent(4);
    auto a2 = Agent::createAgent(5);
    auto o = std::shared_ptr<MySimpleObjective>(new MySimpleObjective(2));

    ASSERT_EQ(o->id(), 2);

    ASSERT_FALSE(o->isDone(a1));
    ASSERT_FALSE(o->isDone(a2));

    o->react(1.0, a1);

    ASSERT_TRUE(o->isDone(a1));
    ASSERT_FALSE(o->isDone(a2));

    o->react(1.0, a2);

    ASSERT_TRUE(o->isDone(a1));
    ASSERT_TRUE(o->isDone(a2));
}
