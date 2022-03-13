
#include <gtest/gtest.h>
#include "objective.h"

TEST(Objective, ConstructDefault)
{
    auto a1 = Agent::createAgent(4);
    auto a2 = Agent::createAgent(5);
    auto o = std::shared_ptr<Objective>(new Objective(2, {a1,a2}));

    ASSERT_EQ(o->id(), 2);
    ASSERT_EQ(o->agents().size(), 2);
    ASSERT_EQ(o->agents().at(0).lock()->id(), 4);
    ASSERT_EQ(o->agents().at(1).lock()->id(), 5);
}
