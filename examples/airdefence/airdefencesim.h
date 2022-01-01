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

#ifndef AD_SIM_H
#define AD_SIM_H

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

#include "environment.h"
#include "missile_station.h"


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

        auto t = Agent::createAgent(6);
        t->setPosition(Eigen::Vector2d(3.0, 7.0));
        t->setVelocity(Eigen::Vector2d(0.0, -2.0));
        t->setRadius(0.3);

        auto m = std::shared_ptr<MissileStation>(new MissileStation(2000, 1, 4.01, 2.4));
        m->setPosition(Eigen::Vector2d(-1.0, 0.0), true);
        m->setRadius(0.2, true);

        agents.push_back(t);
        agents.push_back(m);

        return agents;
    }
};

#endif //AD_SIM_H
