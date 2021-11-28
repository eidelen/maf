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
#include "evaluation.h"

std::shared_ptr<Simulation> Simulation::createSimulation(unsigned int id)
{
    return std::shared_ptr<Simulation>(new Simulation(id));
}

Simulation::Simulation(unsigned int id): m_id(id), m_simulationRunningTime(0.0)
{
    // set default dummy evaluation
    m_evaluation = std::shared_ptr<Evaluation>(new Evaluation());
    m_description = "";
}

Simulation::~Simulation()
{

}

unsigned int Simulation::id() const
{
    return m_id;
}

void Simulation::setDescription(const std::string &desc)
{
    m_description = desc;
}

std::string Simulation::description() const
{
    return m_description;
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

void Simulation::setEvaluation(const std::shared_ptr<Evaluation>& eval)
{
    m_evaluation = eval;
}

std::shared_ptr<Evaluation> Simulation::getEvaluation()
{
    return m_evaluation;
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
        m_environment->addAgent(a);
    });
}

void Simulation::doTimeStep(double time)
{
    m_simulationRunningTime += time;
    m_environment->update(time);
    m_evaluation->evaluate(shared_from_this(), time);
}

std::shared_ptr<Environment> Simulation::getEnvironment()
{
    return m_environment;
}

double Simulation::getSimulationRunningTime() const
{
    return m_simulationRunningTime;
}

void Simulation::runSimulation(double timeStep, double simulationDuration)
{
    while( m_simulationRunningTime < simulationDuration )
    {
        doTimeStep(timeStep);
    }
}
