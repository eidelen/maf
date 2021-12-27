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

#include "missile_station.h"

MissileStation::MissileStation(unsigned int id, size_t nMissiles, double detectionRange): Agent(id)
{
    // create sensor
    m_sensor.reset(new ProximitySensor(++id, detectionRange));
    m_sensor->addIgnoreAgentId(this->id());
    addSubAgent(m_sensor);

    // create rockets
    for(size_t k = 0; k < nMissiles; k++)
    {
        auto missile = std::shared_ptr<Missile>(new Missile(++id));
        m_missiles.push(missile);
        m_sensor->addIgnoreAgentId(missile->id());
        addSubAgent(missile);
    }
}

MissileStation::~MissileStation()
{

}

MissileStation::Status MissileStation::status() const
{
    if(m_missiles.empty())
    {
        return Empty;
    }
    else
    {
        return Operate;
    }
}

void MissileStation::update(double time)
{

}

AgentType MissileStation::type() const
{
    return AgentType::EMissileStation;
}
