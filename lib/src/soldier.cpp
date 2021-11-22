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

#include "soldier.h"
#include "helpers.h"

std::shared_ptr<Soldier> Soldier::createSoldier(unsigned int id, double maxSpeed, double maxAcceleration, double obsDistance, double reactionTime)
{
    return std::shared_ptr<Soldier>(new Soldier(id, maxSpeed, maxAcceleration, obsDistance, reactionTime));
}

Soldier::Soldier(unsigned int id, double maxSpeed, double maxAcceleration, double obsDistance, double reactionTime) : Human(id, maxSpeed, maxAcceleration, obsDistance, reactionTime)
{

}

Soldier::~Soldier()
{

}

void Soldier::react(double time)
{
    // React on neighbours but keep going towards objective

    // Compute target direction
    bool hasTarget = false;
    Eigen::Vector2d targetDirNorm(0.0, 0.0);
    if(!m_objectives.empty())
    {
        Eigen::Vector2d target = m_objectives.front();
        targetDirNorm = (target - getPosition()).normalized();
        hasTarget = true;
    }

    // check if reacting on neigbhours
    auto[compPossible, avgAgentDir] = MafHlp::computeAvgWeightedDirectionToOtherAgents(getEnvironment()->getAgentDistancesToAllOtherAgents(id()), m_obsDistance);

    if( compPossible && hasTarget )
    {
        setAcceleration((-avgAgentDir + targetDirNorm).normalized() * m_maxAccelreation);
    }
    else if(compPossible)
    {
        setAcceleration(-(avgAgentDir) * m_maxAccelreation);
    }
    else if(hasTarget)
    {
        setAcceleration(targetDirNorm* m_maxAccelreation);
    }
    else
    {
        // deaccelerate over reaction time
        setAcceleration(MafHlp::computeSlowDown(m_velocity, m_maxAccelreation, m_reactionTime));
    }
}

void Soldier::addObjective(const Eigen::Vector2d &target)
{
    m_objectives.push(target);
}
