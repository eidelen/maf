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

#include "proximity_sensor.h"


std::shared_ptr<ProximitySensor> ProximitySensor::createProxSensor(unsigned int id, double range)
{
    return std::shared_ptr<ProximitySensor>(new ProximitySensor(id, range));
}

ProximitySensor::ProximitySensor(unsigned int id, double range): Agent::Agent(id), m_range(range)
{

}

ProximitySensor::~ProximitySensor()
{

}

void ProximitySensor::update(double time)
{
    // update first sub agents
    updateSubAgents(time);

    assert(hasEnvironment());

    // update agents in range
    m_agentsInRange.clear();
    EnvironmentInterface::DistanceQueue q = m_environment.lock()->getAgentDistancesToAllOtherAgents(id());
    while(!q.empty())
    {
        const auto& d = q.top();

        // if target is not on ignore list and target is in range
        if( m_ignoreAgentIds.find(d.targetId) == m_ignoreAgentIds.end() &&
                d.dist < m_range )
        {
            m_agentsInRange.push_back(d);
            q.pop();
        }
        else
        {
            // DistanceQueue is ordered -> next agent is out of range too
            break;
        }
    }

    performMove(time);
}

AgentType ProximitySensor::type() const
{
    return AgentType::EProxSensor;
}

double ProximitySensor::range() const
{
    return m_range;
}

void ProximitySensor::setRange(double newRange)
{
    m_range = newRange;
}

std::vector<EnvironmentInterface::Distance> ProximitySensor::getAgentsInSensorRange() const
{
    return m_agentsInRange;
}

void ProximitySensor::addIgnoreAgentId(unsigned int agentId)
{
    m_ignoreAgentIds.insert(agentId);
}

