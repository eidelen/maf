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

#include "slowdown.h"
#include "agent.h"
#include "helpers.h"

SlowDown::SlowDown(unsigned int id, int priority, std::weak_ptr<Agent> agent) : Objective(id, priority, agent)
{

}

SlowDown::~SlowDown()
{

}

void SlowDown::react(double timeStep)
{

    std::shared_ptr<Agent> agent = m_agent.lock();
    agent->setAcceleration(MafHlp::computeSlowDown(agent->getVelocity(), agent->accelreationLimit(), timeStep));
}

bool SlowDown::isDone() const
{
    return (m_agent.lock()->getVelocity().norm() < 0.001);
}
