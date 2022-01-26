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
#include "plane.h"

/**
 * @brief Open space environment
 */
class PlaneEnv: public Environment
{
public:
    PlaneEnv(unsigned int id): Environment(id) {}
    virtual ~PlaneEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        return {true, destination};
    }
};

class PlaneEnvFactory: public EnvironmentFactory
{
public:
    PlaneEnvFactory(): EnvironmentFactory() {}
    virtual ~PlaneEnvFactory() {}

    virtual std::shared_ptr<Environment> createEnvironment() override
    {
        return std::shared_ptr<PlaneEnv>(new PlaneEnv(99));
    }
};


class AirdefenceAgentFactory: public AgentFactory
{
public:
    AirdefenceAgentFactory(): AgentFactory() {}
    virtual ~AirdefenceAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        std::list<std::shared_ptr<Agent>> agents;

        auto p = std::shared_ptr<Plane>(new Plane(10000));
        p->setPosition(Eigen::Vector2d(3.0, 7.0));
        p->setVelocity(Eigen::Vector2d(0.0, -2.0));
        p->setRadius(0.3);

        auto b = std::shared_ptr<Plane>(new Plane(10001));
        b->setPosition(Eigen::Vector2d(2.0, 7.0));
        b->setVelocity(Eigen::Vector2d(-1.5, -1.1));
        b->setRadius(0.3);

        auto m = std::shared_ptr<MissileStation>(new MissileStation(2000, 2, 4.01, 2.4));
        m->setPosition(Eigen::Vector2d(-1.0, 0.0), true);
        m->setRadius(0.2, true);

        agents.push_back(m);
        agents.push_back(p);
        agents.push_back(b);


        return agents;
    }
};

#endif //AD_SIM_H
