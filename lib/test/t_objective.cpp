
#include <gtest/gtest.h>
#include "objective.h"

TEST(Objective, ConstructDefault)
{
    auto a1 = Agent::createAgent(4);
    auto a2 = Agent::createAgent(5);
    auto o = std::shared_ptr<Objective>(new Objective(2, {a1, a2}));

    ASSERT_EQ(o->id(), 2);
    ASSERT_EQ(o->agents().size(), 2);
    ASSERT_EQ(o->agents().at(0).lock()->id(), 4);
    ASSERT_EQ(o->agents().at(1).lock()->id(), 5);
}

class MySimpleObjective: public Objective
{
public:
    MySimpleObjective(unsigned int id, std::vector<AgentWP> agents) : Objective(id, agents)
    {
    }

    void react(double timeStep) override
    {
        for(auto a: m_agents)
        {
            a.lock()->setPosition(Eigen::Vector2d(2.0, 4.0));
        }
    }

    bool isDone() const override
    {
        return std::all_of(m_agents.begin(), m_agents.end(), [](AgentWP a){
           return (a.lock()->getPosition() - Eigen::Vector2d(2.0, 4.0)).isMuchSmallerThan(0.0001);
        });
    }
};


TEST(Objective, SimpleObjective)
{
    auto a1 = Agent::createAgent(4);
    auto a2 = Agent::createAgent(5);
    auto o = std::shared_ptr<MySimpleObjective>(new MySimpleObjective(2, {a1, a2}));

    ASSERT_EQ(o->id(), 2);
    ASSERT_EQ(o->agents().size(), 2);
    ASSERT_EQ(o->agents().at(0).lock()->id(), 4);
    ASSERT_EQ(o->agents().at(1).lock()->id(), 5);

    ASSERT_FALSE(o->isDone());

    o->react(1.0);

    ASSERT_TRUE(o->isDone());
}
