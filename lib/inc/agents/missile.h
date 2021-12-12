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

#ifndef MISSILE_H
#define MISSILE_H

#include "agent.h"


class Missile: public Agent
{

public:

    Missile(unsigned int id);

    /**
     * Destructor
     */
    virtual ~Missile();

    /**
     * Fire the missile. Missile will follow target agent with full speed.
     * @param targetId Id of target agent.
     */
    void fire(unsigned int targetId);

    /**
     * Get the target id.
     * @return Target agent id.
     */
    unsigned int target() const;

    enum Status
    {
        Idle,  /** Missile not launched yet */
        Launched, /** Missile launched */
        Detonated /** Missile detonated */
    };

    /**
     * Get status of the missile.
     * @return Status.
     */
    Status status() const;


public: // inherited from Agent
    void update(double time) override;
    AgentType type() const override;

protected:
    unsigned int m_target;
    Status m_status;
    Eigen::Vector2d m_targetPosBefore;
    bool m_targetPosBeforeAvailable;

};

#endif // MISSILE_H
