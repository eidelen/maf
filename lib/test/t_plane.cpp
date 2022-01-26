
#include <gtest/gtest.h>
#include "plane.h"

TEST(Plane, Type)
{
    auto p = std::shared_ptr<Plane>(new Plane(333));
    ASSERT_EQ(p->type(), AgentType::EPlane);
    ASSERT_EQ(p->id(), 333);
}
