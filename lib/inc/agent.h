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

#ifndef AGENT_H
#define AGENT_H

#include <memory>
#include <list>
#include <Eigen/Dense>

#include "environment_interface.h"
#include "objective.h"

/**
 * @brief The AgentType holds agent type identifiers. With
 * every new agent class, this structs needs to be extended.
 */
enum AgentType
{
    EAgent,
    EHuman,
    ESoldier,
    EProxSensor,
    EMissile,
    EMissileStation,
    EPlane,
    EPlaneHostile,
    ETarget
};


/**
 * @brief The Agent class is a base class implementing basic physical
 * laws, such as motion, and basic interaction with the environment.
 */
class Agent
{

public:

    static std::shared_ptr<Agent> createAgent(unsigned int id);

    /**
     * Constructor
     * @param id Agent id.
     */
    Agent(unsigned int id);

    /**
     * Destructor
     */
    virtual ~Agent();

    /**
     * Compute agent's motion after specific time.
     * @param time Time in second
     * @return Pair of position and speed.
     */
    virtual std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMotion(double time) const;

    /**
     * Get the agent's id.
     * @return Agent id.
     */
    unsigned int id() const;

    /**
     * Return type of agent.
     * @return Type string.
     */
    virtual AgentType type() const;

    /**
     * Get the radius of the agent.
     * @return radius in m.
     */
    double getRadius() const;

    /**
     * Set the radius of the agent.
     * @param The radius of the agent in m.
     * @param includeSubAgents If true, subagents are updated too.
     */
    void setRadius(double radius, bool includeSubAgents = false);

    /**
     * Get agent's position.
     * @return Position in m.
     */
    Eigen::Vector2d getPosition() const;

    /**
     * Set agent's position.
     * @param position Position in m.
     * @param includeSubAgents If true, subagents are updated too.
     */
    void setPosition(const Eigen::Vector2d &position, bool includeSubAgents = false);

    /**
     * Get velocity vector in m/s.
     * @return Velocity vector
     */
    Eigen::Vector2d getVelocity() const;

    /**
     * Set velocity vector in m/s.
     * @param velocity in m/s
     */
    virtual void setVelocity(const Eigen::Vector2d &velocity);

    /**
     * Get agent's acceleration m/s^2
     * @return Acceleration
     */
    Eigen::Vector2d getAcceleration() const;

    /**
     * Set agent's acceleration m/s^2. Accelreation vector
     * will be checked and corrected against the allowed max.
     * acceleration.
     * @param Acceleration
     */
    virtual void setAcceleration(const Eigen::Vector2d& acceleration);

    /**
     * Sets agent's maximum acceleration towards given direction.
     * @param accelerationDirection Acceleration direction (magnitude does not matter).
     */
    virtual void setMaxAccelerationInDirection(const Eigen::Vector2d& accelerationDirection);

    /**
     * Sets agent's maximum velocity towards given direction.
     * @param velocityDirection Velocity direction (magnitude does not matter).
     */
    virtual void setMaxVelocityInDirection(const Eigen::Vector2d& velocityDirection);

    /**
     * Moves at a given speed towards given target.
     * @param target Target position.
     * @param velocity Speed m/s
     */
    virtual void setMovingTowardsTarget(const Eigen::Vector2d& target, double velocity);

    /**
     * Set the agent's environment.
     * @param env Environment.
     */
    virtual void setEnvironment(std::shared_ptr<EnvironmentInterface> env);

    /**
     * Get the agent's environment.
     * @return Environment.
     */
    std::weak_ptr<EnvironmentInterface> getEnvironment() const;

    /**
     * Checks if the agent has an environment.
     * @return True or false.
     */
    bool hasEnvironment() const;

    /**
     * Update the agent within the environment for a given time step.
     * Overwrite with specific agent behavior.
     * @param time Time step in s.
     */
    virtual void update(double time);

    /**
     * Perform the actual spatial move of the agent. This
     * function should be called at the end of update.
     * @param time Time step in seconds.
     */
    virtual void performMove(double time);

    /**
     * Adds a sub agent to this agent.
     * @param a Sub agent.
     */
    virtual void addSubAgent(std::shared_ptr<Agent> a);

    /**
     * Get the list of direct sub agents.
     * @return List of sub agents.
     */
    std::list<std::shared_ptr<Agent> > &getSubAgents();

    /**
     * Recursively gets a list of ALL sub agents.
     * @return List of all subagents.
     */
    std::list<std::shared_ptr<Agent>> getAllSubAgents();

    /**
     * Set and get max speed of agent in m/s. DoubleMax at init.
     */
    double velocityLimit() const;
    void setVelocityLimit(double newMaxSpeed);

    /**
     * Set and get max acceleration of agent in m/s^2. DoubleMax at init.
     */
    double accelreationLimit() const;
    void setAccelreationLimit(double newMaxAccelreation);

    /**
     * Handle messages from environment. Overwrite function
     * for specific beahviour.
     */
    virtual void processMessages();

    /**
     * Sends a message to given recepient
     * @param receiverId Receiver agent id
     * @param subject Message subject.
     * @param textParam Text parameter
     * @param vecDoubleParam Vector of double parameter.
     * @param vecIntParam Vector of Int parameter.
     */
    void sendMessage(unsigned int receiverId, Message::Subject subject, const std::string& textParam = "",
                     const std::vector<double>& vecDoubleParam = std::vector<double>(),
                     const std::vector<int>& vecIntParam = std::vector<int>());

    /**
     * @return Agent disabled or enabled.
     */
    virtual bool getEnabled() const;

    /**
     * Enable or disable the agent.
     * @param enabled
     */
    void setEnabled(bool enabled);

    /**
     * Add an objecitve.
     * @param objective SP to objective.
     */
    void addObjective(ObjectiveSP objective);

protected:

    void updateSubAgents(double time);

    /**
     * Overwrite in derived classes, but if message unknown, ecalate it to parent class.
     */
    virtual void processMessage(std::shared_ptr<Message> msg);

    unsigned int m_id;
    double m_radius;
    Eigen::Vector2d m_position;
    Eigen::Vector2d m_velocity;
    Eigen::Vector2d m_acceleration;

    std::weak_ptr<EnvironmentInterface> m_environment;

    std::list<std::shared_ptr<Agent>> m_subAgents;

    double m_maxSpeed;
    double m_maxAccelreation;

    double m_enabled;

    ObjectivePriorityQueue m_objectives;
};

using AgentSP = std::shared_ptr<Agent>;
using AgentWP = std::weak_ptr<Agent>;

/**
 * @brief The AgentFactory class is the base class for further
 * derived Agent classes.
 */
class AgentFactory
{
public:
    AgentFactory() {}
    virtual ~AgentFactory() {}

    /**
     * Overwrite function for specific agent.
     * @return Generic Agent.
     */
    virtual std::list<std::shared_ptr<Agent>> createAgents()
    {
        // Default just 1 agent
        std::list<std::shared_ptr<Agent>> agents;
        agents.push_back( Agent::createAgent(6) );
        return agents;
    }

private:
    unsigned int m_cnt;
};


#endif // AGENT_H
