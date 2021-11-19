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
#include <random>
#include <Eigen/Dense>
#include <QPainter>

#include "environment.h"
#include "human.h"
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


class CivilianAgentFactory: public AgentFactory
{
public:
    CivilianAgentFactory(): AgentFactory() {}
    virtual ~CivilianAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        std::list<std::shared_ptr<Agent>> agents;

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> reactionDist{0.7, 0.2};

        size_t sideNbr = 12;
        unsigned int agentIdx = 0;
        for(size_t m = 0; m < sideNbr; m++)
        {
            for(size_t n = 0; n < sideNbr; n++)
            {
                auto h1 = std::shared_ptr<Human>(new Human(agentIdx++, 1.0, 2.0, 1.5, reactionDist(gen)));
                h1->setPosition(Eigen::Vector2d(-3.0, -3.0) + m * Eigen::Vector2d(0.5, 0.0) + n * Eigen::Vector2d(0.0, 0.5) );
                agents.push_back(h1);
            }
        }

        return agents;
    }
};

class HumanoidAgentQtSim
{
public:
    HumanoidAgentQtSim()
    {
        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<CivilianAgentFactory>(new CivilianAgentFactory()));
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

    QPointF sim2WidTrans(const Eigen::Vector2d& simCoord)
    {
        Eigen::Vector2d circOrigin(500.0, 400.0);
        Eigen::Vector2d widCoord = (simCoord * sim2WidScale(1.0)) + circOrigin;
        return QPointF(widCoord(0), widCoord(1));
    }

    double sim2WidScale(double simLength)
    {
        return simLength * 30.0;
    }

    void drawSim(QPainter& painter)
    {
        // draw circle
        painter.setBrush(Qt::white);
        painter.drawEllipse(sim2WidTrans(Eigen::Vector2d(0.0, 0.0)), sim2WidScale(10.0), sim2WidScale(10.0));

        // draw agents
        auto agents = m_sim->getEnvironment()->getAgents();
        std::for_each(agents.begin(), agents.end(), [=, &painter](const auto& a) {
            painter.setBrush(Qt::blue);
            painter.drawEllipse(sim2WidTrans(a->getPosition()), sim2WidScale(a->getRadius()), sim2WidScale(a->getRadius()));

            painter.setPen(QPen(Qt::black, 1, Qt::SolidLine));
            painter.drawLine(sim2WidTrans(a->getPosition()), sim2WidTrans(a->getPosition() + (a->getAcceleration()/2.0)));
        });
    }

private:
    double m_timeStep;
    std::shared_ptr<Simulation> m_sim;

};


#endif //CL_SIM_H
