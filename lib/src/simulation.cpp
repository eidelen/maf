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

#include "simulation.h"

std::shared_ptr<Simulation> Simulation::createSimulation(unsigned int id)
{
    return std::shared_ptr<Simulation>(new Simulation(id));
}

Simulation::Simulation(unsigned int id): m_id(id)
{

}

Simulation::~Simulation()
{

}

unsigned int Simulation::id() const
{
    return m_id;
}

std::shared_ptr<AgentFactory> Simulation::agentFactory() const
{
    return m_agentFactory;
}

void Simulation::setAgentFactory(const std::shared_ptr<AgentFactory> &agentFactory)
{
    m_agentFactory = agentFactory;
}

std::shared_ptr<EnvironmentFactory> Simulation::environmentFactory() const
{
    return m_environmentFactory;
}

void Simulation::setEnvironmentFactory(const std::shared_ptr<EnvironmentFactory> &environmentFactory)
{
    m_environmentFactory = environmentFactory;
}

void Simulation::initEnvironment()
{
    m_environment = m_environmentFactory->createEnvironment();
}

void Simulation::initAgents()
{
    auto agents = m_agentFactory->createAgents();
    std::for_each(agents.begin(), agents.end(), [=] (std::shared_ptr<Agent>& a)
    {
        a->setEnvironment(m_environment);
        m_agents.push_back(a);
    });
}

void Simulation::doTimeStep(double time)
{
    // update the agents
    std::for_each(m_agents.begin(), m_agents.end(), [time](std::shared_ptr<Agent>& a)
    {
        a->move(time);
    });
}

std::list<std::shared_ptr<Agent> > &Simulation::getAgents()
{
    return m_agents;
}

std::shared_ptr<Environment> Simulation::getEnvironment()
{
    return m_environment;
}

std::pair<std::vector<unsigned int>, Eigen::MatrixXd> Simulation::getAgentDistanceMap()
{
    return m_agentDistanceMap;
}

void Simulation::computeDistanceMap()
{

}

Eigen::Vector2d Simulation::computeDistance(const std::shared_ptr<Agent> &a, const std::shared_ptr<Agent> &b)
{
    return b->getPosition() - a->getPosition();
}


