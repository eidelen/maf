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

#include "maintain_distance.h"
#include "helpers.h"
#include "agent.h"

MaintainDistance::MaintainDistance(unsigned int id, int priority, std::weak_ptr<Agent> agent, double obsDistance)
    : Objective(id, priority, agent), m_observationDistance(obsDistance)
{

}

MaintainDistance::~MaintainDistance()
{

}

void MaintainDistance::react(double timeStep)
{
    // React on neighbours. When no neigbhours, slow down.

    std::shared_ptr<Agent> agent = m_agent.lock();

    // Compute mean direction of agents in range
    auto[compPossible, avgAgentDir] = MafHlp::computeAvgWeightedDirectionToOtherAgents(agent->getEnvironment().lock()->getAgentDistancesToAllOtherAgents(agent->id()), m_observationDistance);

  /*
    // Compute collision with environment
    std::vector<std::pair<double, Eigen::Vector2d>> envBorderDistances = agent->getEnvironment().lock()->circularSamplingDistancesToEnvironmentBorder(agent->getPosition(), 8, 0.2, m_observationDistance);

    // compute weighted avg
    Eigen::Vector2d bestDirectionAwayFromEnvBorder(0.0, 0.0);
    for( std::pair<double, Eigen::Vector2d> sample : envBorderDistances)
    {
        bestDirectionAwayFromEnvBorder = bestDirectionAwayFromEnvBorder + (sample.first * sample.second);
    }

    setMaxAccelerationInDirection(bestDirection);
    auto[p, v] = computeMotion(time);
    auto[possible, finalPos] = env->possibleMove(getPosition(), p);
    if(possible)
    {
        setPosition(finalPos);
        setVelocity(v);
        return;
    }*/



    if(compPossible)
    {
        agent->setMaxAccelerationInDirection(-avgAgentDir);
    }
    else
    {
        // stop within time resolution with max acceleration

        agent->setAcceleration(MafHlp::computeSlowDown(agent->getVelocity(), agent->accelreationLimit(), timeStep));
    }
}

bool MaintainDistance::isDone() const
{
    // never finishes
    return false;
}
