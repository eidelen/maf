
#include <gtest/gtest.h>

#include "simulation.h"

TEST(Simulation, Id)
{
    auto s = Simulation::createSimulation(4);
    ASSERT_EQ(s->id(), 4);
}

TEST(Simulation, Factories)
{
    auto s = Simulation::createSimulation(4);

    ASSERT_FALSE(s->agentFactory().get() != nullptr);
    ASSERT_FALSE(s->environmentFactory().get() != nullptr);

    s->setAgentFactory(std::shared_ptr<AgentFactory>(new AgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<EnvironmentFactory>(new EnvironmentFactory()));

    ASSERT_TRUE(s->agentFactory().get() != nullptr);
    ASSERT_TRUE(s->environmentFactory().get() != nullptr);
}


