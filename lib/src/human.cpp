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
#include "human.h"


std::shared_ptr<Human> Human::createHuman(unsigned int id, double maxSpeed, double maxAcceleration)
{
    return std::shared_ptr<Human>(new Human(id, maxSpeed, maxAcceleration));
}

Human::Human(unsigned int id, double maxSpeed, double maxAcceleration) :
    Agent(id), m_maxSpeed(maxSpeed), m_maxAccelreation(maxAcceleration)
{

}

Human::~Human()
{

}



std::pair<Eigen::Vector2d, Eigen::Vector2d> Human::computeMotion(double time) const
{
    std::cout << "computeMotion " <<  getVelocity().transpose() << "  ,   " << getAcceleration().transpose() << std::endl;
    Eigen::Vector2d newSpeed = correctVectorScale( getVelocity() + getAcceleration()*time, m_maxSpeed);
    std::cout << "new Speed " <<  newSpeed.transpose()  << std::endl;
    Eigen::Vector2d relevantSpeed = (getVelocity() + newSpeed)/2.0;
    Eigen::Vector2d newPos = getPosition() + relevantSpeed * time;
    std::cout << "new Pos " <<  newPos.transpose()  << std::endl;

    return {newPos, newSpeed};
}

void Human::setVelocity(const Eigen::Vector2d &velocity)
{
    m_velocity = correctVectorScale(velocity, m_maxSpeed);
}

void Human::setAcceleration(const Eigen::Vector2d &acceleration)
{
    m_acceleration = correctVectorScale(acceleration, m_maxAccelreation);
}

Eigen::Vector2d Human::correctVectorScale(const Eigen::Vector2d &in, double maxMagnitude) const
{
    double inLength = in.norm();
    if( inLength > 1.0e-10 && inLength > maxMagnitude ) // do not correct small vectors
    {
        return (in / inLength) * maxMagnitude;
    }

    return in;
}

