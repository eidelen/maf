
#include <gtest/gtest.h>
#include "objective.h"
#include "agent.h"
#include "maintain_distance.h"
#include "move_to_target.h"
#include "slowdown.h"
#include "environment.h"

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


//************************* Maintain Distance ****************************//


TEST(MaintainDist, SlowDown)
{
    auto env = Environment::createEnvironment(3);
    auto a = Agent::createAgent(11);

    a->setVelocityLimit(10.0);
    a->setAccelreationLimit(4.0);
    a->setEnvironment(env);

    double obsDist = 3.0;
    auto m = std::shared_ptr<MaintainDistance>(new MaintainDistance(1, 10, a, obsDist));

    a->addObjective(m);

    // MaintainDistance-agent with 10 m/s in x direction
    a->setVelocity(Eigen::Vector2d(10.0, 0.0));
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));

    // MaintainDistance-agent gets slower, as no other agent around
    double lastSpeed = a->getVelocity().norm();

    a->update(1.0);
    ASSERT_GT(lastSpeed, a->getVelocity().norm());
    lastSpeed = a->getVelocity().norm();

    a->update(1.0);
    ASSERT_GT(lastSpeed, a->getVelocity().norm());
    lastSpeed = a->getVelocity().norm();

    a->update(1.0);
    ASSERT_GT(lastSpeed, a->getVelocity().norm());
}

TEST(MaintainDist, MoveAwayFromOtherHuman)
{
    auto env = Environment::createEnvironment(3);

    auto a = Agent::createAgent(0);
    a->setEnvironment(env);
    a->setVelocityLimit(10.0);
    a->setAccelreationLimit(10.0);

    double obsDist = 10.0;
    auto m = std::shared_ptr<MaintainDistance>(new MaintainDistance(1, 10, a, obsDist));
    a->addObjective(m);

    // dummy does not care
    auto dummy = Agent::createAgent(1);
    dummy->setEnvironment(env);

    env->addAgent(dummy);
    env->addAgent(a);


    // Dont care about a second agent outside the obsDistance
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    dummy->setPosition(Eigen::Vector2d(-20.0, 0.0));
    env->update(1.0);
    ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((dummy->getPosition() - Eigen::Vector2d(-20.0, 0.0)).isMuchSmallerThan(0.0001));

    // a moves away from dummy in x-direction
    dummy->setPosition(Eigen::Vector2d(-1.0, 0.0));
    double lastDist = 1.0;

    while(lastDist < 10.0) // After 10m h1 does not care and slows down.
    {
        env->update(0.1);

        // dummy stays
        ASSERT_TRUE((dummy->getPosition() - Eigen::Vector2d(-1.0, 0.0)).isMuchSmallerThan(0.0001));
        double dist = (dummy->getPosition().transpose() - a->getPosition().transpose()).norm();

        //ASSERT_GT(dist, lastDist);
        lastDist = dist;
    }
}


//************************* Move to target ****************************//

TEST(MoveToTarget, Basic)
{
    auto env = Environment::createEnvironment(3);
    auto a = Agent::createAgent(11);

    a->setVelocityLimit(1.0);
    a->setAccelreationLimit(1.0);
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->setEnvironment(env);

    auto m = std::shared_ptr<MoveToTarget>(new MoveToTarget(1, 10, a, Eigen::Vector2d(0.0, 10.0), 0.2));

    a->addObjective(m);

    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    // full speed after 1 second towards target
    a->update(1.0);
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 1.0)).isMuchSmallerThan(0.0001));
    ASSERT_FALSE(m->isDone());

    // till done
    while( !m->isDone() )
    {
        a->update(0.1);
    }

    // agent close target pos
    ASSERT_LE((a->getPosition() - Eigen::Vector2d(0.0, 10.0)).norm(), 0.2001);

    // agent continous with same speed after reach goal
    a->update(2.0);
    ASSERT_GE((a->getPosition() - Eigen::Vector2d(0.0, 10.0)).norm(), 1.0);

    a->update(1.0);
    ASSERT_GE((a->getPosition() - Eigen::Vector2d(0.0, 10.0)).norm(), 2.0);

    a->update(1.0);
    ASSERT_GE((a->getPosition() - Eigen::Vector2d(0.0, 10.0)).norm(), 3.0);
}

//************************* Move to target and stop ****************************//

TEST(Stop, MoveAndStop)
{
    auto env = Environment::createEnvironment(3);
    auto a = Agent::createAgent(11);

    a->setVelocityLimit(1.0);
    a->setAccelreationLimit(1.0);
    a->setPosition(Eigen::Vector2d(0.0, 0.0));
    a->setEnvironment(env);

    auto m = std::shared_ptr<MoveToTarget>(new MoveToTarget(1, 10, a, Eigen::Vector2d(0.0, 10.0), 0.2));
    auto s = std::shared_ptr<SlowDown>(new SlowDown(1, 11, a));

    a->addObjective(m);
    a->addObjective(s);

    ASSERT_FALSE(m->isDone());

    // first move to target
    while( !m->isDone() )
    {
        a->update(0.01);
    }

    // now slow down happens
    a->update(0.01); // pops move objective
    a->update(0.01);

    // max deacceleration
    ASSERT_TRUE((a->getAcceleration() - Eigen::Vector2d(0.0, -1.0)).isMuchSmallerThan(0.0001));

    while( !s->isDone() )
    {
        a->update(0.01);
    }

    // no speed when finished
    ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.001));
}
