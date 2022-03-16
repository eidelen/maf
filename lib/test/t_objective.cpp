
#include <gtest/gtest.h>
#include "objective.h"
#include "agent.h"

TEST(Objective, ConstructDefault)
{
    auto a1 = Agent::createAgent(4);
    auto o = std::shared_ptr<Objective>(new Objective(2, 33, a1));
    ASSERT_EQ(o->id(), 2);
    ASSERT_EQ(o->priority(), 33);
}

class MySimpleObjective: public Objective
{
public:
    MySimpleObjective(unsigned int id, int prio, AgentWP agent) : Objective(id, prio, agent)
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
    auto o = std::shared_ptr<MySimpleObjective>(new MySimpleObjective(2, 44, a1));

    ASSERT_EQ(o->id(), 2);
    ASSERT_EQ(o->priority(), 44);

    ASSERT_FALSE(o->isDone());

    o->react(1.0);

    ASSERT_TRUE(o->isDone());
}

TEST(Objective, PrioQueue)
{
    ObjectivePriorityQueue q;

    auto a1 = Agent::createAgent(4);
    auto obj0 = std::shared_ptr<Objective>(new Objective(1, 10, a1));
    auto obj1 = std::shared_ptr<Objective>(new Objective(2, 9, a1));
    auto obj2 = std::shared_ptr<Objective>(new Objective(3, 11, a1));

    ASSERT_EQ(q.size(), 0);

    q.push(obj0); q.push(obj1); q.push(obj2);

    ASSERT_EQ(q.size(), 3);

    ASSERT_EQ(q.top()->id(), 2);
    q.pop();
    ASSERT_EQ(q.top()->id(), 1);
    q.pop();
    ASSERT_EQ(q.top()->id(), 3);
    q.pop();

    ASSERT_TRUE(q.empty());
}

TEST(Objective, PopWhenDone)
{
    auto a1 = Agent::createAgent(4);
    auto o0 = std::shared_ptr<Objective>(new Objective(1, 10, a1));
    auto o1 = std::shared_ptr<MySimpleObjective>(new MySimpleObjective(2, 11, a1));

    ObjectivePriorityQueue q;
    q.push(o0);
    q.push(o1);

    ASSERT_EQ(q.size(), 2);

    // o0 default objective imp is immediatly done
    q.popWhenDone();

    ASSERT_EQ(q.size(), 1);

    // nothing happens
    q.popWhenDone();

    ASSERT_EQ(q.size(), 1);

    // once reacting is enough
    q.top()->react(1.0);

    // nothing happens
    q.popWhenDone();

    ASSERT_EQ(q.size(), 0);
}
