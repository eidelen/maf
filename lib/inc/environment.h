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

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <memory>
#include <list>
#include <vector>
#include <Eigen/Dense>

#include "environment_interface.h"
#include "agent.h"

/**
 * @brief The Environment class is base class representing the agent's
 * environment, respectiveley an interface.
 */
class Environment: public EnvironmentInterface
{

public:

    static std::shared_ptr<Environment> createEnvironment(unsigned int id);

    /**
     * Constructor
     * @param id Environment id.
     */
    Environment(unsigned int id);

    /**
     * Destructor
     */
    virtual ~Environment();

    /**
     * Get id of environment.
     * @return Id
     */
    unsigned int id() const;

    /**
     * Update environment by a time step. This function
     * can be overwritten with specific functionality.
     * @param time Time in second.
     */
    virtual void update(double time);

    /**
     * Add an agent to the environment.
     * @param a Agent
     */
    void addAgent(std::shared_ptr<Agent> a);

    /**
     * Get a handle to all agents in the simulation.
     * @return List of agents.
     */
    std::list<std::shared_ptr<Agent>>& getAgents();

    /**
     * Get the agents distance map.
     * @return pair of a vector, which assigns the agent ids to the matrix row, and
     * a matrix with distances between each agent to each other agent.
     */
    std::pair<std::vector<unsigned int>, Eigen::MatrixXd> getAgentDistanceMap();

    /**
     * Compute the distances between each agent to each other agent. The
     * distance map can be accessed with getAgentDistanceMap().
     */
    void computeDistanceMap();

    /**
     * Compute the distance vector between two agents;
     * @param a Agent 1
     * @param b Agent 2
     * @return Distance vector.
     */
    static Eigen::Vector2d computeDistance(const std::shared_ptr<Agent>& a, const std::shared_ptr<Agent>& b);


public: //Inherited from EnvironmentInterface

    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override;


private:

    unsigned int m_id;
    std::list<std::shared_ptr<Agent>> m_agents;
    std::pair<std::vector<unsigned int>, Eigen::MatrixXd> m_agentDistanceMap;
};


/**
 * @brief The EnvironmentFactory class is the base class for further
 * derived Environment classes.
 */
class EnvironmentFactory
{
public:
    EnvironmentFactory() : m_cnt(0) {}
    virtual ~EnvironmentFactory() {}

    /**
     * Overwrite function for specific environment.
     * @return Generic Environment.
     */
    virtual std::shared_ptr<Environment> createEnvironment()
    {
        return Environment::createEnvironment(m_cnt++);
    }

private:
    unsigned int m_cnt;
};


#endif // ENVIRONMENT_H