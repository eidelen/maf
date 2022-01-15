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

#ifndef MISSILE_STATION_H
#define MISSILE_STATION_H

#include <set>
#include <queue>
#include "agent.h"
#include "missile.h"
#include "proximity_sensor.h"

/**
 * @brief The MissileStation class represents a missile station with a
 * proximity sensor and a limited number of missiles.
 */
class MissileStation: public Agent
{

public:

    /**
     * @brief MissileStation
     * @param id Missile station id.
     * @param nMissiles Number of missiles.
     * @param detectionRange Distance in which an object is detected.
     * @param missileVelocity Missile velocity.
     */
    MissileStation(unsigned int id, size_t nMissiles, double detectionRange, double missileVelocity);

    /**
     * Destructor
     */
    virtual ~MissileStation();

    enum Status
    {
        Operate,  /** Station operates */
        Empty /** No missiles left */
    };

    /**
     * Get status of the missile.
     * @return Status.
     */
    Status status() const;

    /**
     * Get the detection range in m.
     * @return Detection range in m.
     */
    double detectionRange() const;


public: // inherited from Agent
    void update(double time) override;
    AgentType type() const override;
    void setEnvironment(std::shared_ptr<EnvironmentInterface> env) override;

protected:
    std::set<unsigned int> m_targets;
    std::queue<std::shared_ptr<Missile>> m_missiles;
    std::shared_ptr<ProximitySensor> m_sensor;
    double m_detectionRange;
};

#endif // MISSILE_STATION_H
