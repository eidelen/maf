
#include <gtest/gtest.h>

#include "simulation.h"
#include "evaluation.h"

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

TEST(Simulation, InitEnvAgent)
{
    auto s = Simulation::createSimulation(4);
    s->setAgentFactory(std::shared_ptr<AgentFactory>(new AgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<EnvironmentFactory>(new EnvironmentFactory()));

    ASSERT_FALSE(s->getEnvironment().get() != nullptr);

    s->initEnvironment();
    s->initAgents();

    ASSERT_TRUE(s->getEnvironment().get() != nullptr);
    ASSERT_EQ(s->getEnvironment()->getAgents().size(), 1);
}

class CircEnv: public Environment
{
public:
    CircEnv(unsigned int id): Environment(id) {}
    virtual ~CircEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        // Circular environmet with radius 10. If move not possible, return
        // previous position.
        double radius = 10.0;
        if(destination.norm() < radius)
            return {true, destination};
        else
            return {false, origin};
    }
};

class CircEnvFactory: public EnvironmentFactory
{
public:
    CircEnvFactory(): EnvironmentFactory() {}
    virtual ~CircEnvFactory() {}

    virtual std::shared_ptr<Environment> createEnvironment() override
    {
        return std::shared_ptr<CircEnv>(new CircEnv(99));
    }
};

class MyBoringAgent: public Agent
{
public:
    MyBoringAgent(unsigned int k): Agent(k)
    {
        setPosition(Eigen::Vector2d(0.0, 0.0));
        setVelocity(Eigen::Vector2d(1.0, 0.0));
    }
    virtual ~MyBoringAgent() {}
};

class MyBoringAgentFactory: public AgentFactory
{
public:
    MyBoringAgentFactory(): AgentFactory() {}
    virtual ~MyBoringAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        // create 2 agents
        return {std::shared_ptr<MyBoringAgent>(new MyBoringAgent(0)),
                    std::shared_ptr<MyBoringAgent>(new MyBoringAgent(1))};
    }
};

TEST(Simulation, TestBaseSimulation)
{
    auto s = Simulation::createSimulation(4);
    s->setAgentFactory(std::shared_ptr<MyBoringAgentFactory>(new MyBoringAgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

    s->initEnvironment();
    s->initAgents();

    ASSERT_TRUE(s->getEnvironment().get() != nullptr);
    ASSERT_EQ(s->getEnvironment()->getAgents().size(), 2);

    // MyBoringAgent is created a position 0,0 and moves with 1m/s in x-axis direction

    // Time zero -> velocity 1, pos 0,0
    auto myAgents = s->getEnvironment()->getAgents();
    std::for_each(myAgents.begin(), myAgents.end(), [](std::shared_ptr<Agent>& a){
        ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
        ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    });

    // Time 2s -> velocity 1, pos 2,0
    s->doTimeStep(2.0);
    auto myAgents1 = s->getEnvironment()->getAgents();
    std::for_each(myAgents1.begin(), myAgents1.end(), [](std::shared_ptr<Agent>& a){
        ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(2.0, 0.0)).isMuchSmallerThan(0.0001));
        ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    });

    // Time 5s -> velocity 1, pos 3,0
    s->doTimeStep(3.0);
    auto myAgents2 = s->getEnvironment()->getAgents();
    std::for_each(myAgents2.begin(), myAgents2.end(), [](std::shared_ptr<Agent>& a){
        ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));
        ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    });

    // Time 11s -> outside, not possible. Position stays as before
    s->doTimeStep(6.0);
    auto myAgents3 = s->getEnvironment()->getAgents();
    std::for_each(myAgents3.begin(), myAgents3.end(), [](std::shared_ptr<Agent>& a){
        ASSERT_TRUE((a->getPosition() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));
        ASSERT_TRUE((a->getVelocity() - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));
    });
}

TEST(Simulation, RunningTime)
{
    auto s = Simulation::createSimulation(4);
    s->setAgentFactory(std::shared_ptr<MyBoringAgentFactory>(new MyBoringAgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));
    s->initEnvironment();
    s->initAgents();

    ASSERT_NEAR(s->getSimulationRunningTime(), 0.0, 0.0001);

    s->doTimeStep(0.5);

    ASSERT_NEAR(s->getSimulationRunningTime(), 0.5, 0.0001);

    s->doTimeStep(2.5);

    ASSERT_NEAR(s->getSimulationRunningTime(), 3.0, 0.0001);
}

TEST(Simulation, RunSimulation)
{
    auto s = Simulation::createSimulation(4);
    s->setAgentFactory(std::shared_ptr<MyBoringAgentFactory>(new MyBoringAgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));
    s->initEnvironment();
    s->initAgents();

    ASSERT_NEAR(s->getSimulationRunningTime(), 0.0, 0.0001);

    s->runSimulation(0.05, 10.0);

    ASSERT_NEAR(s->getSimulationRunningTime(), 10.0, 0.0001);
}

// Acculates overall agent living time
class MySumAgentEvaluation: public Evaluation
{
public:
    MySumAgentEvaluation(): Evaluation() { m_sumAgentsLivingtime = 0.0; }
    virtual ~MySumAgentEvaluation() {}
    void evaluate(std::shared_ptr<Simulation> env, double timeStep) override
    {
        m_sumAgentsLivingtime += env->getEnvironment()->getAgents().size() * timeStep;
    }

    std::string getResult() override
    {
        return std::to_string(m_sumAgentsLivingtime);
    }

    double m_sumAgentsLivingtime;
};

TEST(Simulation, Evaluation)
{
    auto s = Simulation::createSimulation(4);
    s->setAgentFactory(std::shared_ptr<MyBoringAgentFactory>(new MyBoringAgentFactory()));
    s->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

    auto eval = std::shared_ptr<MySumAgentEvaluation>(new MySumAgentEvaluation());
    s->setEvaluation(eval);

    s->initEnvironment();
    s->initAgents();

    // the agent factory creates 2 agents

    // at beginning, the overall living time is 0
    ASSERT_NEAR(eval->m_sumAgentsLivingtime, 0.0, 0.00001);
    ASSERT_EQ(eval->getResult(), std::to_string(0.0));

    // after 1s -> 2s
    s->doTimeStep(1.0);
    ASSERT_NEAR(eval->m_sumAgentsLivingtime, 2.0, 0.00001);
    ASSERT_EQ(eval->getResult(), std::to_string(2.0));

    // 4 -> 8
    s->doTimeStep(3.0);
    ASSERT_NEAR(eval->m_sumAgentsLivingtime, 8.0, 0.00001);
    ASSERT_EQ(eval->getResult(), std::to_string(8.0));
}
