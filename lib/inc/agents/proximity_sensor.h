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

#ifndef POXIMITY_SENS_H
#define POXIMITY_SENS_H

#include <vector>

#include "agent.h"

/**
 * @brief This class is sensor agent, which checks if another
 * agent comes within a certain range.
 */
class ProximitySensor: public Agent
{

public:

    static std::shared_ptr<ProximitySensor> createProxSensor(unsigned int id, double range);

    /**
     * Ctor of Proximity Sensor. The sensor area is circular shape.
     * @param id Sensor Agent Id
     * @param range The distance range the sensor gets active, in meter.
     */
    ProximitySensor(unsigned int id, double range);

    /**
     * Destructor
     */
    virtual ~ProximitySensor();

    /**
     * Get the proximity sensor range in meter.
     * @return senor range.
     */
    double range() const;

    /**
     * Set the proximity sensor range in meter.
     * @param newRange Range in meter.
     */
    void setRange(double newRange);

    /**
     * Get the agents in sensor range ordered in increasing distance.
     * @return Agents.
     */
    std::vector<EnvironmentInterface::Distance> getAgentsInSensorRange() const;



public: // inherited from Agent
    void update(double time) override;
    AgentType type() const override;


protected:
    double m_range;
    std::vector<EnvironmentInterface::Distance> m_agentsInRange;

};



#endif // POXIMITY_SENS_H
