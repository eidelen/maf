
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

    //TODO: addAgent needs to add also subagents
    //NOTE: update calls subagents -> therefore subagents would be updated twice
    //Note: proximity detects itself now :(

    auto a = Agent::createAgent(1);
    a->setPosition(Eigen::Vector2d(0.0, 6.0));
    a->setEnvironment(e);
    e->addAgent(a);

    for(auto q: e->getAgents())
    {
        std::cout << q->id() << std::endl;
    }

    e->update(0.1);

    // the only single rocket NOT fired yet
    ASSERT_EQ(s->status(), MissileStation::Operate);

    a->setPosition(Eigen::Vector2d(0.0, 4.0));



    // the single rocket was fired -> no rocket left
    ASSERT_EQ(s->status(), MissileStation::Empty);

}
