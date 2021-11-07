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

#include "agent.h"

std::shared_ptr<Agent> Agent::createAgent(unsigned int id)
{
    return std::shared_ptr<Agent>(new Agent(id));
}

Agent::Agent(unsigned int id) : m_id(id), m_radius(0.0), m_velocity(Eigen::Vector2d(0.0, 0.0)), m_acceleration(Eigen::Vector2d(0.0, 0.0))
{

}

Agent::~Agent()
{

}

std::pair<Eigen::Vector2d, Eigen::Vector2d> Agent::computeMotion(double time) const
{
    auto newSpeed = getVelocity() + getAcceleration()*time;
    auto relevantSpeed = (getVelocity() + newSpeed)/2.0;
    auto newPos = getPosition() + relevantSpeed * time;

    std::cout << "time: " << time << std::endl;
    std::cout << "addSpeed: " << (getAcceleration() * time).transpose() << std::endl;
    std::cout << "newSpeed: " << newSpeed.transpose() << std::endl;
    std::cout << "relevantSpeed: " << relevantSpeed.transpose() << std::endl;
    std::cout << "acceleration: " << getAcceleration().transpose() << std::endl;
    std::cout << "oldSpeed: " << getVelocity().transpose() << std::endl;
    std::cout << "oldPos: " << getPosition().transpose() << std::endl;

    return {newPos, newSpeed};
}

unsigned int Agent::id() const
{
    return m_id;
}

double Agent::getRadius() const
{
    return m_radius;
}

void Agent::setRadius(double radius)
{
    m_radius = radius;
}

Eigen::Vector2d Agent::getPosition() const
{
    return m_position;
}

void Agent::setPosition(const Eigen::Vector2d &position)
{
    m_position = position;
}

Eigen::Vector2d Agent::getVelocity() const
{
    return m_velocity;
}

void Agent::setVelocity(const Eigen::Vector2d &velocity)
{
    m_velocity = velocity;
}

Eigen::Vector2d Agent::getAcceleration() const
{
    return m_acceleration;
}

void Agent::setAcceleration(const Eigen::Vector2d& acceleration)
{
    m_acceleration = acceleration;
}
