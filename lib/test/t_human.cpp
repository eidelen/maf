
#include <gtest/gtest.h>
#include "human.h"
#include "environment.h"

TEST(Human, Type)
{
    auto a = Human::createHuman(11, 10.0, 1.0, 3.0, -1.0);
    ASSERT_EQ(a->type(), AgentType::EHuman);
}
