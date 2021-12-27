
#include <gtest/gtest.h>
#include "missile_station.h"

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
