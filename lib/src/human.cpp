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
#include "human.h"
#include "helpers.h"


std::shared_ptr<Human> Human::createHuman(unsigned int id, double maxSpeed,
                                          double maxAcceleration, double obsDistance)
{
    return std::shared_ptr<Human>(new Human(id, maxSpeed, maxAcceleration, obsDistance));
}

Human::Human(unsigned int id, double maxSpeed, double maxAcceleration, double obsDistance) :
    Agent(id), m_maxSpeed(maxSpeed), m_maxAccelreation(maxAcceleration), m_obsDistance(obsDistance),
    m_disableReacting(false)
{

}

Human::~Human()
{

}

void Human::disableReacting(bool disable)
{
    m_disableReacting = disable;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Human::computeMotion(double time) const
{
    Eigen::Vector2d newSpeed = MafHlp::correctVectorScale( getVelocity() + getAcceleration()*time, m_maxSpeed);
    Eigen::Vector2d relevantSpeed = (getVelocity() + newSpeed)/2.0;
    Eigen::Vector2d newPos = getPosition() + relevantSpeed * time;

    return {newPos, newSpeed};
}

void Human::setVelocity(const Eigen::Vector2d &velocity)
{
    m_velocity = MafHlp::correctVectorScale(velocity, m_maxSpeed);
}

void Human::setAcceleration(const Eigen::Vector2d &acceleration)
{
    m_acceleration = MafHlp::correctVectorScale(acceleration, m_maxAccelreation);
}

void Human::move(double time)
{
    assert(hasEnvironment());

    auto env = getEnvironment();

    if( !m_disableReacting )
    {
        EnvironmentInterface::DistanceQueue agentDistances = env->getAgentDistancesToAllOtherAgents(id());

        // React on neighbours. When no neigbhours, slow down.
        Eigen::Vector2d newAcceleration;

        // Consider agents
        if(!agentDistances.empty())
        {
            EnvironmentInterface::Distance closestAgent = agentDistances.top();
            if(closestAgent.dist < m_obsDistance)
            {
                //                  unit direction pointing away from other agent
                newAcceleration = -(closestAgent.vect / closestAgent.dist) * m_maxAccelreation;
            }
            else
            {
                newAcceleration = MafHlp::computeSlowDown(m_velocity, m_maxAccelreation, time);
            }
        }
        else
        {
            newAcceleration = MafHlp::computeSlowDown(m_velocity, m_maxAccelreation, time);
        }

        setAcceleration(newAcceleration);
    }


    // Update new velocity and position
    auto[p, v] = computeMotion(time);
    auto[possible, finalPos] = env->possibleMove(getPosition(), p);
    setPosition(finalPos);
    setVelocity(possible ? v : getVelocity()); // only update velocity when motion was possible
}

