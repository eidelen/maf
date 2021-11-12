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

#ifndef CL_SIM_H
#define CL_SIM_H

#include <iostream>
#include <Eigen/Dense>
#include <QPainter>

#include "environment.h"
#include "agent.h"
#include "simulation.h"

/**
 * @brief Trivial environment with circle 10m radius and center at 0/0
 */
class CircEnv: public Environment
{
public:
    CircEnv(unsigned int id): Environment(id) {}
    virtual ~CircEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        // Circular environmet with radius 10m. If move not possible, return
        // previous position.
        double radius = 10.0;
        if(destination.norm() < radius)
            return {true, destination};
        else
            return {false, origin};
    }
};

class CircEnvFactory: public EnvironmentFactory
{
public:
    CircEnvFactory(): EnvironmentFactory() {}
    virtual ~CircEnvFactory() {}

    virtual std::shared_ptr<Environment> createEnvironment() override
    {
        return std::shared_ptr<CircEnv>(new CircEnv(99));
    }
};

class HumanoidAgent: public Agent
{
public:
    HumanoidAgent(unsigned int k): Agent(k)
    {
        setPosition(Eigen::Vector2d(0.0, 0.0));
        setVelocity(Eigen::Vector2d(1.0, 0.0));
    }
    virtual ~HumanoidAgent() {}
};

class HumanoidAgentFactory: public AgentFactory
{
public:
    HumanoidAgentFactory(): AgentFactory() {}
    virtual ~HumanoidAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        // return one agent
        return {std::shared_ptr<HumanoidAgent>(new HumanoidAgent(0))};
    }
};

class HumanoidAgentQtSim
{
public:
    HumanoidAgentQtSim()
    {
        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<HumanoidAgentFactory>(new HumanoidAgentFactory()));
        m_sim->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

        m_sim->initEnvironment();
        m_sim->initAgents();
    }

    virtual ~HumanoidAgentQtSim() {}

    void setTimeStep(double ts)
    {
        m_timeStep = ts;
    }

    void update()
    {
        m_sim->doTimeStep(m_timeStep);
    }

    void drawSim(QPainter& painter)
    {
        double meterToPixel = 30.0;
        Eigen::Vector2d circOrigin(500.0, 400.0);

        // draw circle
        painter.setBrush(Qt::white);
        painter.drawEllipse(QPointF(circOrigin(0), circOrigin(1)), meterToPixel*10.0, meterToPixel*10.0);

        // draw agents
        auto agents = m_sim->getAgents();
        std::for_each(agents.begin(), agents.end(), [&painter, circOrigin, meterToPixel](const auto& a) {

            Eigen::Vector2d pPix = (a->getPosition() * meterToPixel) + circOrigin;
            painter.setBrush(Qt::blue);
            painter.drawEllipse(QPointF(pPix(0), pPix(1)), meterToPixel*0.3, meterToPixel*0.3);

        });
    }



private:
    double m_timeStep;
    std::shared_ptr<Simulation> m_sim;

};


#endif //CL_SIM_H
