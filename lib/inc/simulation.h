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

#ifndef SIMULATION_H
#define SIMULATION_H

#include <memory>
#include <list>
#include <vector>

#include "agent.h"
#include "environment.h"

class Evaluation;

class Simulation : public std::enable_shared_from_this<Simulation>
{

public:

    static std::shared_ptr<Simulation> createSimulation(unsigned int id);

    /**
     * Constructor for simulation.
     * @param id Simulation id.
     */
    Simulation(unsigned int id);

    /**
     * Destructor.
     */
    virtual ~Simulation();

    /**
     * Get Simulation id.
     * @return Id.
     */
    unsigned int id() const;

    /**
     * Get the agent factory.
     * @return Agent factory
     */
    std::shared_ptr<AgentFactory> agentFactory() const;

    /**
     * Sets the agent factory.
     * @param agentFactory An agent factory.
     */
    void setAgentFactory(const std::shared_ptr<AgentFactory> &agentFactory);

    /**
     * Get the environment factory.
     * @return Environment factory.
     */
    std::shared_ptr<EnvironmentFactory> environmentFactory() const;

    /**
     * Sets the environment factory.
     * @param environmentFactory Environment factory.
     */
    void setEnvironmentFactory(const std::shared_ptr<EnvironmentFactory> &environmentFactory);

    /**
     * Sets the evaluation class.
     * @param eval Evaluation instance.
     */
    void setEvaluation(const std::shared_ptr<Evaluation>& eval);

    /**
     * Inits the environment. No agents added yet.
     */
    void initEnvironment();

    /**
     * Creates and add the agents.
     */
    void initAgents();

    /**
     * Run the simulation for specific time.
     * @param time Time in seconds.
     */
    void doTimeStep(double time);

    /**
     * Get the environment.
     * @return Environment
     */
    std::shared_ptr<Environment> getEnvironment();

    /**
     * Return the overall simulation running time in seconds.
     * @return Running time in seconds.
     */
    double getSimulationRunningTime() const;

    /**
     * Run simulation for a given time with given time steps.
     * @param timeStep Time steps in seconds.
     * @param simulationDuration Simulation duration in seconds.
     */
    void runSimulation(double timeStep, double simulationDuration);

private:
    unsigned int m_id;
    double m_simulationRunningTime;
    std::shared_ptr<AgentFactory> m_agentFactory;
    std::shared_ptr<EnvironmentFactory> m_environmentFactory;
    std::shared_ptr<Environment> m_environment;
    std::shared_ptr<Evaluation> m_evaluation;
};


#endif // SIMULATION_H
