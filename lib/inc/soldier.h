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

#ifndef SOLDIER_H
#define SOLDIER_H

#include <queue>
#include "human.h"

class Soldier: public Human
{

public:

    static std::shared_ptr<Soldier> createSoldier(unsigned int id, double maxSpeed = 2.0,
                                                    double maxAcceleration = 1.0, double obsDistance = 1.5,
                                                    double reactionTime = 0.3);

    Soldier(unsigned int id, double maxSpeed, double maxAcceleration, double obsDistance, double reactionTime);

    /**
     * Destructor
     */
    virtual ~Soldier();

    /**
     * React. Specify behaviour in this function.
     */
    virtual void react(double time) override;

    /**
     * Add an objective / target location, the soldier
     * should move to.
     * @param target Target location.
     */
    void addObjective(const Eigen::Vector2d& target);

protected:
    std::queue<Eigen::Vector2d> m_objectives;

};



#endif // SOLDIER_H
