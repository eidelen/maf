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

#include "environment.h"

#define _USE_MATH_DEFINES
#include <math.h>

std::shared_ptr<Environment> Environment::createEnvironment(unsigned int id)
{
    return std::shared_ptr<Environment>(new Environment(id));
}

Environment::Environment(unsigned int id) : m_id(id), m_enableLogMessages(true)
{

}

Environment::~Environment()
{

}

unsigned int Environment::id() const
{
    return m_id;
}

void Environment::update(double time)
{
    // update the distance map -> agent move will access this map further down
    computeDistances();

    // update the agents -> note: subagents are updated from their parent agent
    std::for_each(m_agents.begin(), m_agents.end(), [time](std::shared_ptr<Agent>& a)
    {
        a->update(time);
    });
}

void Environment::addAgent(std::shared_ptr<Agent> a)
{
    m_agents.push_back(a);
}

std::list<std::shared_ptr<Agent> > Environment::getAgents()
{
    std::list<std::shared_ptr<Agent>> collect;

    // collect all agents and their subagents
    std::for_each(m_agents.begin(), m_agents.end(), [&collect](std::shared_ptr<Agent>& a)
    {
        collect.push_back(a);
        auto z = a->getAllSubAgents();
        collect.insert(collect.end(), z.begin(), z.end());
    });

    return collect;
}

std::pair<bool, Eigen::Vector2d> Environment::possibleMove(const Eigen::Vector2d& /*origin*/, const Eigen::Vector2d& destination) const
{
    return {true, destination};
}

EnvironmentInterface::DistanceQueue Environment::getAgentDistancesToAllOtherAgents(unsigned int id)
{
    return getAgentDistances()[id];
}

Environment::DistanceMap& Environment::getAgentDistances()
{
    return m_agentDistanceMap;
}

void Environment::computeDistances()
{
    m_agentDistanceMap.clear();

    const auto& agents = getAgents();

    // O( n * log(n) )
    for(auto fromA = agents.begin(); fromA != agents.end(); fromA++)
    {
        const auto& agentA = *fromA;

        if(agentA->getEnabled()) // Only consider enabled agents
        {
            for(auto toA = std::next(fromA,1); toA != agents.end(); toA++)
            {
                const auto& agentB = *toA;

                if(agentB->getEnabled())
                {
                    Eigen::Vector2d vDiff = Environment::computeDistance(agentA, agentB);
                    double vLength = vDiff.norm();

                    // extend matrix with distances in both direction
                    m_agentDistanceMap[agentA->id()].push({vLength, agentB->id(), vDiff});
                    m_agentDistanceMap[agentB->id()].push({vLength, agentA->id(), -vDiff});
                }
            }
        }
    }
}

Eigen::Vector2d Environment::computeDistance(const std::shared_ptr<Agent> &a, const std::shared_ptr<Agent> &b)
{
    return b->getPosition() - a->getPosition();
}

void Environment::setEnableLogMessages(bool enable)
{
    m_enableLogMessages = enable;
}

EnvironmentInterface::MessageQueue &Environment::getMessages(unsigned int receiverAgendId)
{
    return m_msgMap[receiverAgendId];
}

void Environment::sendMessage(std::shared_ptr<Message> aMessage)
{
    m_msgMap[aMessage->receiverId()].push(aMessage);
}

void Environment::log(const std::string &logMsg)
{
    if(m_enableLogMessages)
    {
        std::cout << logMsg << std::endl;
    }
}

double Environment::distanceToEnvironmentBorder(const Eigen::Vector2d &pos, const Eigen::Vector2d &dir, double stepSize, double maxDist)
{
    Eigen::Vector2d dirStep = dir.normalized() * stepSize;
    Eigen::Vector2d currentPos = pos;
    double dist = 0.0;
    double lastValidDist = 0.0;

    while (dist < maxDist)
    {
        auto[possible, newPos] = possibleMove(pos, currentPos);
        if( possible )
        {
            currentPos = currentPos + dirStep;
            lastValidDist = dist;
            dist += stepSize;
        }
        else
        {
            return lastValidDist;
        }
    }

    return maxDist;
}

std::vector<std::pair<double, Eigen::Vector2d> > Environment::circularSamplingDistancesToEnvironmentBorder(const Eigen::Vector2d &pos, unsigned int nbrOfSamples,
                                                                                                           double stepSize, double maxDist)
{
    std::vector<std::pair<double, Eigen::Vector2d>> sampledDistances;
    sampledDistances.reserve(nbrOfSamples);

    Eigen::Vector2d baseDirection(1.0, 0.0);
    double angularStep = M_PI * 2.0 / nbrOfSamples;

    for(unsigned int i = 0; i < nbrOfSamples; i++)
    {
        Eigen::Rotation2Dd t(angularStep * i);
        Eigen::Vector2d direction = (t.toRotationMatrix() * baseDirection);

        double dist = distanceToEnvironmentBorder(pos, direction, stepSize, maxDist);

        sampledDistances.emplace_back(dist, direction);
    }

    return sampledDistances;
}
