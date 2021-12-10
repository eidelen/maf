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
                                          double maxAcceleration, double obsDistance,
                                          double reactionTime)
{
    return std::shared_ptr<Human>(new Human(id, maxSpeed, maxAcceleration, obsDistance, reactionTime));
}

Human::Human(unsigned int id, double maxSpeed, double maxAcceleration, double obsDistance, double reactionTime) :
    Agent(id), m_obsDistance(obsDistance),
    m_disableReacting(false), m_reactionTime(reactionTime), m_timeSinceLastReaction(0.0), m_stressLevel(0.0)
{
    setRadius(0.3);
    m_maxSpeed = maxSpeed;
    m_maxAccelreation = maxAcceleration;
}

Human::~Human()
{

}

AgentType Human::type() const
{
    return AgentType::EHuman;
}

void Human::disableReacting(bool disable)
{
    m_disableReacting = disable;
}

double Human::getStressLevel() const
{
    return m_stressLevel;
}

void Human::computeStressLevel(EnvironmentInterface::DistanceQueue otherAgents)
{
    // get closest agent and weight with obsDistance
    double stress = 0.0;
    if(!otherAgents.empty())
    {
        stress = (-1.0/m_obsDistance*otherAgents.top().dist) + 1.0;
    }

    m_stressLevel = std::max(0.0, std::min(1.0, stress));
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Human::computeMotion(double time) const
{
    Eigen::Vector2d newSpeed = MafHlp::correctVectorScale( getVelocity() + getAcceleration()*time, m_maxSpeed);
    Eigen::Vector2d relevantSpeed = (getVelocity() + newSpeed)/2.0;
    Eigen::Vector2d newPos = getPosition() + relevantSpeed * time;

    return {newPos, newSpeed};
}

void Human::update(double time)
{
    assert(hasEnvironment());

    // do not navigate in every move
    m_timeSinceLastReaction += time;
    if( m_timeSinceLastReaction < m_reactionTime )
    {
        performFinalMove(time);
        return;
    }
    m_timeSinceLastReaction = 0.0;

    if( !m_disableReacting )
    {
        react(time);
    }

    computeStressLevel(m_environment.lock()->getAgentDistancesToAllOtherAgents(id()));
    performFinalMove(time);
}

void Human::react(double time)
{
    // React on neighbours. When no neigbhours, slow down.

    // Compute mean direction of agents in range
    auto[compPossible, avgAgentDir] = MafHlp::computeAvgWeightedDirectionToOtherAgents(m_environment.lock()->getAgentDistancesToAllOtherAgents(id()), m_obsDistance);

    if(compPossible)
    {
        setAcceleration(-(avgAgentDir) * m_maxAccelreation);
    }
    else
    {
        // deaccelerate over reaction time, as react() will be no called in between.
        setAcceleration(MafHlp::computeSlowDown(m_velocity, m_maxAccelreation, std::max(time, m_reactionTime)));
    }
}

void Human::performFinalMove(double time)
{
    auto env = m_environment.lock();

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
    for(int k = 0; k < 10; k++)
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
    m_stressLevel = 1.0;
}

