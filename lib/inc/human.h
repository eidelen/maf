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

#ifndef HUMAN_H
#define HUMAN_H


#include "agent.h"


/**
 * @brief This class represents a basic human agent:
 *      - Speed limit
 *      - Keeping minimum distance to other human
 */
class Human: public Agent
{

public:

    static std::shared_ptr<Human> createHuman(unsigned int id, double maxSpeed = 5, double maxAcceleration = 2.5);

    /**
     * Constructor
     * @param id Civilian id.
     */
    Human(unsigned int id, double maxSpeed, double maxAcceleration);

    /**
     * Destructor
     */
    virtual ~Human();


public: // inherited from Agent
    std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMotion(double time) const;
    void setVelocity(const Eigen::Vector2d &velocity);
    void setAcceleration(const Eigen::Vector2d &acceleration);

private:
    Eigen::Vector2d correctVectorScale(const Eigen::Vector2d& in, double maxMagnitude) const;


protected:
    double m_maxSpeed;
    double m_maxAccelreation;
};



#endif // HUMAN_H
