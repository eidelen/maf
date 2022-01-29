
#include <gtest/gtest.h>
#include "plane.h"

TEST(Plane, Type)
{
    auto p = std::shared_ptr<Plane>(new Plane(333));
    ASSERT_EQ(p->type(), AgentType::EPlane);
    ASSERT_EQ(p->id(), 333);

    auto h = std::shared_ptr<HostilePlane>(new HostilePlane(444));
    ASSERT_EQ(h->type(), AgentType::EPlaneHostile);
    ASSERT_EQ(h->id(), 444);
}
