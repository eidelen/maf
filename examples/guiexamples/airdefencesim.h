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
#include <set>
#include <iomanip>
#include <Eigen/Dense>

#include "environment.h"
#include "missile_station.h"
#include "plane.h"
#include "target.h"
#include "evaluation.h"
#include "simulation.h"

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
    AirdefenceAgentFactory(double planeSpeed, double missileSpeed): AgentFactory(),
        m_planeSpeed(planeSpeed), m_missileSpeed(missileSpeed) {}
    virtual ~AirdefenceAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        std::list<std::shared_ptr<Agent>> agents;

        Eigen::Vector2d target(0, 0);
        double planeDist = 300000;
        int nPlanes = 32;

        for(int i = 0; i < nPlanes; i++)
        {
            double angle = (3.14 * 2.0 / nPlanes) * i;
            Eigen::Vector2d originPos(planeDist + i*1000, 0.0);
            Eigen::Rotation2Dd t(angle);
            Eigen::Vector2d planePos = (t.toRotationMatrix() * originPos) + target;

            auto hp = std::shared_ptr<HostilePlane>(new HostilePlane(20000+i));
            hp->setPosition(planePos);
            hp->setMovingTowardsTarget(target, m_planeSpeed);
            agents.push_back(hp);
        }

        auto m = std::shared_ptr<MissileStation>(new MissileStation(2000, nPlanes, 50000, m_missileSpeed));
        m->setPosition(Eigen::Vector2d(-70000.0, 0.0), true);
        agents.push_back(m);

        auto n = std::shared_ptr<MissileStation>(new MissileStation(3000, nPlanes, 50000, m_missileSpeed));
        n->setPosition(Eigen::Vector2d(60000.0, -60000.0), true);
        agents.push_back(n);

        auto r = std::shared_ptr<MissileStation>(new MissileStation(4000, nPlanes, 50000, m_missileSpeed));
        r->setPosition(Eigen::Vector2d(18000.0, 75000.0), true);
        agents.push_back(r);

        auto l = std::shared_ptr<MissileStation>(new MissileStation(5000, nPlanes, 50000, m_missileSpeed));
        l->setPosition(Eigen::Vector2d(170000, 0.0), true);
        agents.push_back(l);

        auto trg = Target::createTarget(102, target, 25000.0);
        agents.push_back(trg);

        return agents;
    }

    double m_planeSpeed;
    double m_missileSpeed;
};

// How many planes reached the target area
class ReachEvaluation: public Evaluation
{
public:

    /**
     * @brief ReachEvaluation
     */
    ReachEvaluation(double speedPlane, double speedMissile): Evaluation(),
        m_speedPlane(speedPlane), m_speedMissile(speedMissile)
    {
        m_currentTime = 0.0;
    }

    virtual ~ReachEvaluation()
    {
    }

    void evaluate(std::shared_ptr<Simulation> sim, double timeStep) override
    {
        auto agents = sim->getEnvironment()->getAgents();
        m_computationTime = sim->getComputationTime();

        for(const auto& a : agents)
        {
            if( a->id() == 102)
            {
                Target* t = (Target*)a.get();
                auto agentsInRange = t->getAgentsInSensorRange();
                for(const auto& ar : agentsInRange )
                {
                    // only consider hostile planes
                    if(ar.targetId >= 20000)
                    {
                        // set doesnt care about multiple insertions
                        m_agentsReachedId.insert(ar.targetId);
                    }
                }

                break;
            }
        }

        m_currentTime = sim->getSimulationRunningTime();
    }

    std::string getResult() override
    {
        std::ostringstream s;
        s << std::fixed << std::setprecision( 3 ) << std::setfill( '0' ) <<
        "Time: " << m_currentTime << " s" << std::endl <<
             "Computation Time: " << m_computationTime << " ms" << std::endl <<
             "Speed Planes: " << m_speedPlane << " m/s" << std::endl <<
             "Speed Missiles: " << m_speedMissile << " m/s" << std::endl <<
                "Target Hits: " << m_agentsReachedId.size();
        return s.str();
    }

    std::set<unsigned long> m_agentsReachedId;
    double m_currentTime;
    int m_computationTime = 0;
    double m_speedPlane;
    double m_speedMissile;

};

#endif //AD_SIM_H
