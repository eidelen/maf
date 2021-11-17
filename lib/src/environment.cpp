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

std::shared_ptr<Environment> Environment::createEnvironment(unsigned int id)
{
    return std::shared_ptr<Environment>(new Environment(id));
}

Environment::Environment(unsigned int id) : m_id(id)
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
    // update the agents
    std::for_each(m_agents.begin(), m_agents.end(), [time](std::shared_ptr<Agent>& a)
    {
        a->move(time);
    });
}

void Environment::addAgent(std::shared_ptr<Agent> a)
{
    m_agents.push_back(a);
}

std::list<std::shared_ptr<Agent> > &Environment::getAgents()
{
    return m_agents;
}

std::pair<bool, Eigen::Vector2d> Environment::possibleMove(const Eigen::Vector2d& /*origin*/, const Eigen::Vector2d& destination) const
{
    return {true, destination};
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

        for(auto toA = std::next(fromA,1); toA != agents.end(); toA++)
        {
            const auto& agentB = *toA;
            Eigen::Vector2d vDiff = Environment::computeDistance(agentA, agentB);
            double vLength = vDiff.norm();

            // extend matrix with distances in both direction
            m_agentDistanceMap[agentA->id()].push({vLength, agentB->id(), vDiff});
            m_agentDistanceMap[agentB->id()].push({vLength, agentA->id(), -vDiff});
        }
    }
}

Eigen::Vector2d Environment::computeDistance(const std::shared_ptr<Agent> &a, const std::shared_ptr<Agent> &b)
{
    return b->getPosition() - a->getPosition();
}

