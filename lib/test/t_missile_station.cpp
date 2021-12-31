
#include <gtest/gtest.h>
#include "missile_station.h"
#include "environment.h"

TEST(MissileStation, TypeAndId)
{
    auto m = std::shared_ptr<MissileStation>(new MissileStation(4,2,3.0));
    ASSERT_EQ(m->type(), AgentType::EMissileStation);
    ASSERT_EQ(m->id(), 4);
}

TEST(MissileStation, CheckComponents)
{
    auto m = std::shared_ptr<MissileStation>(new MissileStation(4, 2, 3.0));

    auto agents = m->getSubAgents();

    // first subagent is the sensor, second and third are rockets
    ASSERT_EQ(agents.size(), 3);

    ASSERT_EQ(agents.front()->id(), 5);
    ASSERT_EQ(agents.front()->type(), AgentType::EProxSensor);

    auto m1 = std::next(agents.begin());
    auto m2 = std::next(std::next(agents.begin()));
    ASSERT_EQ((*m1)->id(), 6);
    ASSERT_EQ((*m1)->type(), AgentType::EMissile);
    ASSERT_EQ((*m2)->id(), 7);
    ASSERT_EQ((*m2)->type(), AgentType::EMissile);
}

TEST(MissileStation, Status)
{
    auto empty = std::shared_ptr<MissileStation>(new MissileStation(4, 0, 3.0));
    auto loaded = std::shared_ptr<MissileStation>(new MissileStation(4, 1, 3.0));

    ASSERT_EQ(empty->status(), MissileStation::Empty);
    ASSERT_EQ(loaded->status(), MissileStation::Operate);
}

TEST(MissileStation, Fire)
{
    auto e = Environment::createEnvironment(0);
    auto s = std::shared_ptr<MissileStation>(new MissileStation(2000, 1, 5.0));
    s->setEnvironment(e);
    s->setPosition(Eigen::Vector2d(0.0, 0.0));
    e->addAgent(s);

    auto a = Agent::createAgent(1);
    a->setPosition(Eigen::Vector2d(0.0, 6.0));
    a->setEnvironment(e);
    e->addAgent(a);

    e->update(0.1);

    // the only single rocket NOT fired yet
    ASSERT_EQ(s->status(), MissileStation::Operate);

    a->setPosition(Eigen::Vector2d(0.0, 4.0));

    e->update(0.1);

    // the single rocket was fired -> no rocket left
    ASSERT_EQ(s->status(), MissileStation::Empty);
}

TEST(MissileStation, MultipleFire)
{
    auto e = Environment::createEnvironment(0);
    auto s = std::shared_ptr<MissileStation>(new MissileStation(2000, 2, 5.0));
    s->setEnvironment(e);
    s->setPosition(Eigen::Vector2d(0.0, 0.0));
    e->addAgent(s);

    auto a = Agent::createAgent(1);
    a->setPosition(Eigen::Vector2d(0.0, 6.0));
    a->setEnvironment(e);
    e->addAgent(a);

    auto b = Agent::createAgent(2);
    b->setPosition(Eigen::Vector2d(0.0, 6.0));
    b->setEnvironment(e);
    e->addAgent(b);

    e->update(0.1);

    // no rocket fired yet
    ASSERT_EQ(s->status(), MissileStation::Operate);

    // first rocket fired
    a->setPosition(Eigen::Vector2d(0.0, 4.0));
    e->update(0.1);
    ASSERT_EQ(s->status(), MissileStation::Operate);

    // do not fire a second time on same agent
    a->setPosition(Eigen::Vector2d(0.0, 2.0));
    e->update(0.1);
    ASSERT_EQ(s->status(), MissileStation::Operate);


    // second and last rocket fired too
    b->setPosition(Eigen::Vector2d(0.0, -4.0));
    e->update(0.1);
    ASSERT_EQ(s->status(), MissileStation::Empty);
}

TEST(MissileStation, FlyBy)
{
    auto e = Environment::createEnvironment(0);
    auto s = std::shared_ptr<MissileStation>(new MissileStation(2000, 10, 1.0));
    s->setEnvironment(e);
    s->setPosition(Eigen::Vector2d(0.0, 0.0));
    e->addAgent(s);

    for(unsigned int k = 0; k < 10; k++)
    {
        // agent will fly over rocket station
        auto a = Agent::createAgent(k);
        a->setPosition(Eigen::Vector2d(-10.0-k, 0.0));
        a->setVelocity(Eigen::Vector2d(1.0, 0.0));
        a->setEnvironment(e);
        e->addAgent(a);
    }

    ASSERT_EQ(s->status(), MissileStation::Operate);

    // after about 20 seconds, all missiles were fired
    for(int t = 0; t < 50; t++)
    {
        e->update(0.5);
    }

    ASSERT_EQ(s->status(), MissileStation::Empty);
}

