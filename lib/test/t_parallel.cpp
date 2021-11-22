#include <gtest/gtest.h>
#include <chrono>

#include "parallel.h"

TEST(Parallel, Run)
{
    auto p = Parallel::createParallel(2);

    auto[d0, q0] = p->getProgress();
    ASSERT_EQ(d0, 0);
    ASSERT_EQ(q0, 0);

    for(unsigned int k = 0; k < 4; k++ )
    {
        auto s = Simulation::createSimulation(k);
        s->setAgentFactory(std::shared_ptr<AgentFactory>(new AgentFactory()));
        s->setEnvironmentFactory(std::shared_ptr<EnvironmentFactory>(new EnvironmentFactory()));

        p->addSimulation(s, 0.2, 10.0);
    }

    auto[d1, q1] = p->getProgress();
    ASSERT_EQ(d1, 0);
    ASSERT_EQ(q1, 4);

    p->run();

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(2000ms);

    auto[d2, q2] = p->getProgress();
    ASSERT_EQ(d2, 4);
    ASSERT_EQ(q2, 0);
}
