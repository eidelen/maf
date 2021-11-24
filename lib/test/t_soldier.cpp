
#include <gtest/gtest.h>
#include "soldier.h"
#include "environment.h"

TEST(Soldier, Id)
{
    auto s = Soldier::createSoldier(8);
    ASSERT_EQ(s->id(), 8);
}

TEST(Soldier, Type)
{
    auto a = Soldier::createSoldier(8);
    ASSERT_EQ(a->type(), AgentType::ESoldier);
}
