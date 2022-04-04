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
#include <limits>

#include "agent.h"
#include "helpers.h"

std::shared_ptr<Agent> Agent::createAgent(unsigned int id)
{
    return std::shared_ptr<Agent>(new Agent(id));
}

Agent::Agent(unsigned int id) : m_id(id), m_radius(0.0), m_velocity(Eigen::Vector2d(0.0, 0.0)),
    m_position(Eigen::Vector2d(0.0, 0.0)), m_acceleration(Eigen::Vector2d(0.0, 0.0)),
    m_maxSpeed(std::numeric_limits<double>::max()), m_maxAccelreation(std::numeric_limits<double>::max()),
    m_enabled(true)
{

}

Agent::~Agent()
{

}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Agent::computeMotion(double time) const
{
    Eigen::Vector2d newSpeed = MafHlp::correctVectorScale(getVelocity() + getAcceleration()*time, m_maxSpeed);
    Eigen::Vector2d relevantSpeed = (getVelocity() + newSpeed)/2.0;
    Eigen::Vector2d newPos = getPosition() + relevantSpeed * time;
    return {newPos, newSpeed};
}

unsigned int Agent::id() const
{
    return m_id;
}

AgentType Agent::type() const
{
    return AgentType::EAgent;
}

double Agent::getRadius() const
{
    return m_radius;
}

void Agent::setRadius(double radius, bool includeSubAgents)
{
    m_radius = radius;

    if(includeSubAgents)
    {
        for(auto sa: getSubAgents())
        {
            sa->setRadius(radius);
        }
    }
}

Eigen::Vector2d Agent::getPosition() const
{
    return m_position;
}

void Agent::setPosition(const Eigen::Vector2d &position, bool includeSubAgents)
{
    m_position = position;

    if(includeSubAgents)
    {
        for(auto sa: getSubAgents())
        {
            sa->setPosition(position);
        }
    }
}

Eigen::Vector2d Agent::getVelocity() const
{
    return m_velocity;
}

Eigen::Vector2d Agent::getAcceleration() const
{
    return m_acceleration;
}

void Agent::setVelocity(const Eigen::Vector2d &velocity)
{
    m_velocity = MafHlp::correctVectorScale(velocity, m_maxSpeed);
}

void Agent::setAcceleration(const Eigen::Vector2d &acceleration)
{
    m_acceleration = MafHlp::correctVectorScale(acceleration, m_maxAccelreation);
}

void Agent::setMaxAccelerationInDirection(const Eigen::Vector2d& accelerationDirection)
{
    m_acceleration = MafHlp::adjustVectorScale(accelerationDirection, m_maxAccelreation);
}

void Agent::setMaxVelocityInDirection(const Eigen::Vector2d& velocityDirection)
{
    m_velocity = MafHlp::adjustVectorScale(velocityDirection, m_maxSpeed);
}

void Agent::setMovingTowardsTarget(const Eigen::Vector2d &target, double velocity)
{
    Eigen::Vector2d diffVect = target - m_position;
    m_velocity = MafHlp::adjustVectorScale(diffVect, std::min(m_maxSpeed, velocity));
}

void Agent::setAccelerateTowardsTarget(const Eigen::Vector2d& target, double acceleration)
{
    Eigen::Vector2d diffVect = target - m_position;
    m_acceleration = MafHlp::adjustVectorScale(diffVect, std::min(m_maxAccelreation, acceleration));
}

void Agent::setEnvironment(std::shared_ptr<EnvironmentInterface> env)
{
    m_environment = env;
}

std::weak_ptr<EnvironmentInterface> Agent::getEnvironment() const
{
    return m_environment;
}

bool Agent::hasEnvironment() const
{
    return m_environment.lock().get() != nullptr;
}

void Agent::update(double time)
{
    // update first sub agents
    updateSubAgents(time);

    assert(hasEnvironment());

    processMessages();

    if(m_enabled)
    {
        if(!m_objectives.empty())
        {
            m_objectives.top()->process(time);
            m_objectives.popWhenDone();
        }

        performMove(time);
    }
}

void Agent::performMove(double time)
{
    // A very basic default implementation how an agent moves.
    auto[p, v] = computeMotion(time);

    auto[possible, finalPos] = m_environment.lock()->possibleMove(getPosition(), p);

    setPosition(finalPos);
    setVelocity(possible ? v : getVelocity()); // only update velocity when motion was possible
}

void Agent::updateSubAgents(double time)
{
    std::for_each(m_subAgents.begin(), m_subAgents.end(), [time](std::shared_ptr<Agent>& a)
    {
        a->update(time);
    });
}

bool Agent::getEnabled() const
{
    return m_enabled;
}

void Agent::setEnabled(bool enabled)
{
    m_enabled = enabled;
}

void Agent::addObjective(ObjectiveSP objective)
{
    m_objectives.push(objective);
}

double Agent::accelreationLimit() const
{
    return m_maxAccelreation;
}

void Agent::setAccelreationLimit(double newMaxAccelreation)
{
    m_maxAccelreation = newMaxAccelreation;
}

void Agent::processMessages()
{
    // Basic agents handles disable and enable messages.
    auto& messages = m_environment.lock()->getMessages(id());

    while (!messages.empty())
    {
        auto m = messages.front();
        processMessage(m);
        messages.pop();
    }
}

void Agent::processMessage(std::shared_ptr<Message> msg)
{
    switch(msg->subject())
    {
        case Message::Enable:
            setEnabled(true);
            break;

        case Message::Disable:
            setEnabled(false);
            break;

        default:
            break;
    }
}

void Agent::sendMessage(unsigned int receiverId, Message::Subject subject, const std::string &textParam, const std::vector<double> &vecDoubleParam, const std::vector<int> &vecIntParam)
{
    auto m = std::shared_ptr<Message>(new Message(id(), receiverId, subject, textParam, vecDoubleParam, vecIntParam));
    m_environment.lock()->sendMessage(m);
}

double Agent::velocityLimit() const
{
    return m_maxSpeed;
}

void Agent::setVelocityLimit(double newMaxSpeed)
{
    m_maxSpeed = newMaxSpeed;
}

void Agent::addSubAgent(std::shared_ptr<Agent> a)
{
    m_subAgents.push_back(a);
}

std::list<std::shared_ptr<Agent>>& Agent::getSubAgents()
{
    return m_subAgents;
}

std::list<std::shared_ptr<Agent> > Agent::getAllSubAgents()
{
    std::list<std::shared_ptr<Agent>> collect;

    // collect sub agents
    std::for_each(m_subAgents.begin(), m_subAgents.end(), [&collect](std::shared_ptr<Agent>& a)
    {
        collect.push_back(a);

        auto z = a->getSubAgents();
        collect.insert(collect.end(), z.begin(), z.end());
    });

    return collect;
}
