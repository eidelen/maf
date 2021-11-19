#include <gtest/gtest.h>

#include "parallel.h"

TEST(Parallel, Run)
{
    auto p = Parallel::createParallel(4);

    for(unsigned int k = 0; k < 4; k++ )
    {
        auto s = Simulation::createSimulation(k);
        s->setAgentFactory(std::shared_ptr<AgentFactory>(new AgentFactory()));
        s->setEnvironmentFactory(std::shared_ptr<EnvironmentFactory>(new EnvironmentFactory()));

        p->addSimulation(s, 0.2, 10.0);
    }

    p->run();

}
