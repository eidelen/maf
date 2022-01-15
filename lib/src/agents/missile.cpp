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
#include "missile.h"
#include "helpers.h"

Missile::Missile(unsigned int id): Agent(id), m_target(0), m_status(Status::Idle),
    m_targetPosBeforeAvailable(false)
{

}

Missile::~Missile()
{
    
}

void Missile::fire(unsigned int targetId)
{
    m_target = targetId;
    m_status = Status::Launched;
}

unsigned int Missile::target() const
{
    return m_target;
}

Missile::Status Missile::status() const
{
    return m_status;
}

void Missile::update(double time)
{
    // update first sub agents
    updateSubAgents(time);

    assert(hasEnvironment());

    if(m_status == Launched)
    {
        // get target direction
        auto agentDists = m_environment.lock()->getAgentDistancesToAllOtherAgents(id());
        auto[found, dist] = MafHlp::getDistanceToAgent(agentDists, m_target);
        if(found)
        {
            Eigen::Vector2d estimatedTargetDirection = dist.vect;
            Eigen::Vector2d currentTargetPosition = m_position + dist.vect;

            // compute intersection by estimating future target position
            if(m_targetPosBeforeAvailable)
            {
                Eigen::Vector2d targetVelocity = (currentTargetPosition - m_targetPosBefore) / time;

                // how long to fly with current speed
                double timeToReach = dist.dist / m_maxSpeed;

                // estimate where targe will be by then
                Eigen::Vector2d estimatedTargetPosition = currentTargetPosition + targetVelocity * timeToReach;
                estimatedTargetDirection = estimatedTargetPosition - m_position;
            }
            m_targetPosBeforeAvailable = true;
            m_targetPosBefore = currentTargetPosition;

            // max acceleration towards target
            setMaxVelocityInDirection(estimatedTargetDirection);

            // if target closer than missile can fly within "time" -> detonate
            if(dist.dist < time * m_maxSpeed)
            {
                std::cout << "Missile " << id() << " detonated: Target " << m_target << std::endl;

                sendMessage(m_target, Message::Disable);

                m_status = Detonated;
                m_acceleration = Eigen::Vector2d(0.0, 0.0);
                m_velocity = Eigen::Vector2d(0.0, 0.0);
            }
        }
    }

    performMove(time);
}

AgentType Missile::type() const
{
    return AgentType::EMissile;
}
