/****************************************************************************
** Copyright (c) 2021 Adrian Schneider
**
** Permission is hereby granted, free of charge, to any person obtaining a
** copy of this software and associated documentation files (the "Software"),
** to deal in the Software without restriction, including without limitation
** the rights to use, copy, modify, merge, publish, distribute, sublicense,
** and/or sell copies of the Software, and to permit persons to whom the
** Software is furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in
** all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
**
*****************************************************************************/

#include <iostream>

#include "simulation.h"
#include "agent.h"

/**
 * Example using the simulation and environment- and agent-factroies.
 */


/**
 * Implement out own environment -> Circle with r = 10.0
 */
class CircularEnvironment: public Environment
{
public:
    CircularEnvironment(unsigned int id, double radius): Environment(id), m_radius(radius) {}
    virtual ~CircularEnvironment() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        // moves outside circle restricted
        if(destination.norm() < m_radius)
            return {true, destination};
        else
            return {false, origin};
    }
    double m_radius;
};

/**
 * Factory that creates CircularEnvironment
 */
class Ex2EnvFactory : public EnvironmentFactory
{
public:
    Ex2EnvFactory(double radius): EnvironmentFactory(), m_radius(radius) {}
    virtual ~Ex2EnvFactory() {}

    virtual std::shared_ptr<Environment> createEnvironment() override
    {
        return std::shared_ptr<CircularEnvironment>(new CircularEnvironment(99, m_radius));
    }
    double m_radius;
};

/**
 * Creates the agents within the simulation.
 */
class Ex2AgentFactory: public AgentFactory
{
public:
    Ex2AgentFactory(const Eigen::Vector2d& speed): AgentFactory(),
        m_agentSpeed(speed) {}
    virtual ~Ex2AgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        std::list<std::shared_ptr<Agent>> agents;

        // create on agent a set its speed to the one given in the constructor
        auto a = Agent::createAgent(2);
        a->setPosition(Eigen::Vector2d(0.0, 0.0));
        a->setVelocity(m_agentSpeed);
        agents.push_back(a);

        return agents;
    }

    Eigen::Vector2d m_agentSpeed;
};


int main(int argc, char *argv[])
{
    auto sim = Simulation::createSimulation(0);
    sim->setAgentFactory(std::shared_ptr<Ex2AgentFactory>( new Ex2AgentFactory( Eigen::Vector2d(0.0, 1.0) )) );
    sim->setEnvironmentFactory(std::shared_ptr<Ex2EnvFactory>(new Ex2EnvFactory(10.0)));

    // init environment and factory -> Note: No memory allocated before!!
    sim->initEnvironment();
    sim->initAgents();

    std::cout << " 0s: " << sim->getEnvironment()->getAgents().front()->getPosition().transpose() << std::endl;
    sim->runSimulation(0.1, 3.0);
    std::cout << " 3s: " <<sim->getEnvironment()->getAgents().front()->getPosition().transpose() << std::endl;
    sim->runSimulation(0.1, 10.0);
    std::cout << "13s: " <<sim->getEnvironment()->getAgents().front()->getPosition().transpose() << std::endl;
    sim->runSimulation(0.1, 20.0);
    std::cout << "33s: " <<sim->getEnvironment()->getAgents().front()->getPosition().transpose() << std::endl;
}

