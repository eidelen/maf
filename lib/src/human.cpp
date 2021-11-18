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
#include <random>
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
    setRadius(0.3);

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

        // Compute mean direction of agents in range
        Eigen::Vector2d newAcceleration(0.0, 0.0);
        size_t cntAgents = 0;
        while(!agentDistances.empty())
        {
            if( agentDistances.top().dist < m_obsDistance )
            {
                cntAgents++;

                // weigthed summation
                newAcceleration = newAcceleration + (agentDistances.top().vect / agentDistances.top().dist) * 1.0 / cntAgents;
                agentDistances.pop();
            }
            else
            {
                break;
            }
        }

        if(cntAgents > 0)
        {
            newAcceleration = -(newAcceleration / newAcceleration.norm()) * m_maxAccelreation;
        }
        else
        {
            newAcceleration = MafHlp::computeSlowDown(m_velocity, m_maxAccelreation, time);
        }

        setAcceleration(newAcceleration);
    }

    performFinalMove(time);
}

void Human::performFinalMove(double time)
{
    auto env = getEnvironment();

    // try to move with initial settings
    auto[pi, vi] = computeMotion(time);
    auto[initialPossible, resFinalPos] = env->possibleMove(getPosition(), pi);
    if(initialPossible)
    {
        setPosition(resFinalPos);
        setVelocity(vi);
        return;
    }

    // initial move did not work, try something similiar

    // randomly generating angle deviation around wanted direction
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0.0, 3.14};

    Eigen::Vector2d wantedAcceleration = getAcceleration();
    for(int k = 0; k < 5; k++)
    {
        Eigen::Rotation2D<double> rotMat( d(gen) );
        Eigen::Vector2d actualAcceleration = rotMat * wantedAcceleration;

        setAcceleration(actualAcceleration);
        auto[p, v] = computeMotion(time);
        auto[possible, finalPos] = env->possibleMove(getPosition(), p);

        if(possible)
        {
            // found possible random move
            setPosition(finalPos);
            setVelocity(v);
            return;
        }
    }

    // No move found within sampling range
    setVelocity(Eigen::Vector2d(0.0, 0.0));
    setAcceleration(Eigen::Vector2d(0.0, 0.0));
}

