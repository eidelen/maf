
#include <gtest/gtest.h>

#include "simulation.h"

TEST(Simulation, Id)
{
    auto s = Simulation::createSimulation(4);
    ASSERT_EQ(s->id(), 4);
}


