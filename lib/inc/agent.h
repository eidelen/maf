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

#ifndef AGENT_H
#define AGENT_H

#include <memory>
#include <Eigen/Dense>

class Agent
{

public:

    static std::shared_ptr<Agent> createAgent(unsigned int id);

    /**
     * Constructor
     * @param id Agent id.
     */
    Agent(unsigned int id);

    /**
     * Destructor
     */
    virtual ~Agent();

    /**
     * Get the agent's id.
     * @return Agent id.
     */
    unsigned int id() const;

    /**
     * Get the radius of the agent.
     * @return radius in m.
     */
    double getRadius() const;

    /**
     * Set the radius of the agent.
     * @param The radius of the agent in m.
     */
    void setRadius(double radius);

    /**
     * Get agent's position.
     * @return Position in m.
     */
    Eigen::Vector2d getPosition() const;

    /**
     * Set agent's position.
     * @param position Position in m.
     */
    void setPosition(const Eigen::Vector2d &position);

    /**
     * Get velocity vector in m/s.
     * @return Velocity vector
     */
    Eigen::Vector2d getVelocity() const;

    /**
     * Set velocity vector in m/s.
     * @param velocity in m/s
     */
    void setVelocity(const Eigen::Vector2d &velocity);

private:

    unsigned int m_id;
    double m_radius;
    Eigen::Vector2d m_position;
    Eigen::Vector2d m_velocity;
    double m_maxVelocity;
    double m_acceleration;
    double m_maxAcceleration;
};


#endif // AGENT_H
