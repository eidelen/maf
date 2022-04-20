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
#include <iomanip>
#include <random>
#include <Eigen/Dense>

#include "environment.h"
#include "human.h"
#include "simulation.h"
#include "evaluation.h"
#include "shapes.h"
#include "maintain_distance.h"

/**
 * @brief Trivial environment with circle 10m radius and center at 0/0
 */
class CLEnv: public Environment
{
public:
    CLEnv(unsigned int id): Environment(id) {}
    virtual ~CLEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        auto l = {Quadrant::createQuadrant(77, Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(10.0, 1.0)),
                  Quadrant::createQuadrant(78, Eigen::Vector2d(5.0, 1.0), Eigen::Vector2d(7.0, 9.0)),
                   Quadrant::createQuadrant(79, Eigen::Vector2d(9.5, -12.0), Eigen::Vector2d(32.0, 17.0)),
                    Quadrant::createQuadrant(80, Eigen::Vector2d(-10.0, 8.0), Eigen::Vector2d(32.0, 17.0))};

        if( std::any_of(l.begin(), l.end(), [destination](auto thisQuadrant){ return thisQuadrant->isInShape(destination);}) )
            return {true, destination};
        else
            return {false, origin};
    }
};

class CLEnvFactory: public EnvironmentFactory
{
public:
    CLEnvFactory(): EnvironmentFactory() {}
    virtual ~CLEnvFactory() {}

    virtual std::shared_ptr<Environment> createEnvironment() override
    {
        return std::shared_ptr<CLEnv>(new CLEnv(99));
    }
};


class CivilianAgentFactory: public AgentFactory
{
public:
    CivilianAgentFactory(double maxSpeed = 1.0, double maxAcceleration = 1.0): AgentFactory(),
        m_maxSpeed(maxSpeed), m_maxAcceleration(maxAcceleration) {}
    virtual ~CivilianAgentFactory() {}
    std::list<std::shared_ptr<Agent>> createAgents() override
    {
        std::list<std::shared_ptr<Agent>> agents;

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> reactionDist{0.4, 0.2};

        double obsDistance = 1.5;
        size_t sideNbr = 30;
        unsigned int agentIdx = 0;
        for(size_t m = 0; m < sideNbr; m++)
        {
            for(size_t n = 0; n < sideNbr; n++)
            {
                auto h1 = std::shared_ptr<Human>(new Human(agentIdx++, m_maxSpeed, m_maxAcceleration, obsDistance, reactionDist(gen)));
                h1->setPosition(Eigen::Vector2d(0.0, 0.0) + m * Eigen::Vector2d(0.001, 0.0) + n * Eigen::Vector2d(0.0, 0.001) );

                auto behavior = std::shared_ptr<MaintainDistance>(new MaintainDistance(h1->id(), 1, h1, obsDistance));
                h1->addObjective(behavior);

                agents.push_back(h1);
            }
        }

        return agents;
    }

    double m_maxSpeed;
    double m_maxAcceleration;
};

// Acculates overall stress
class StressAccumulatorEvaluation: public Evaluation
{
public:
    /**
     * Passing the used simulation settings.
     * @param maxSpeed
     * @param maxAcceleration
     */
    StressAccumulatorEvaluation(double maxSpeed, double maxAcceleration): Evaluation()
    {
        m_stressSeconds = 0.0;
        m_currentTime = 0.0;
        m_currentStress = 0.0;

        m_maxSpeed = maxSpeed;
        m_maxAcceleration = maxAcceleration;
    }

    virtual ~StressAccumulatorEvaluation()
    {
    }

    void evaluate(std::shared_ptr<Simulation> sim, double timeStep) override
    {
        auto agents = sim->getEnvironment()->getAgents();
        m_computationTime = sim->getComputationTime();

        double avgStress = 0.0;
        int cntAgents = 0;
        int cntActive = 0;
        std::for_each(agents.begin(), agents.end(), [&cntActive, &avgStress, &cntAgents](const auto& a) {
            Human* h = (Human*)a.get();
            double stress = h->getStressLevel();
            avgStress += stress;
            cntAgents++;

            if(h->getEnabled())
            {
                cntActive++;
            }
        });

        avgStress = avgStress / std::max(1, cntAgents);
        m_stressSeconds += timeStep * avgStress;
        m_currentStress = avgStress;
        m_nActiveHuman = cntActive;

        m_currentTime = sim->getSimulationRunningTime();

        // simulate a door by disable agents in this region
        /*auto door = Quadrant::createQuadrant(6666, Eigen::Vector2d(9.0, -1.0), Eigen::Vector2d(11.0, 1.0));
        std::for_each(agents.begin(), agents.end(), [door](const auto& a) {
            Human* h = (Human*)a.get();
            if( door->isInQuadrant(h->getPosition()) )
            {
                h->setEnabled(false);
            }
        });*/

    }

    std::string getResult() override
    {
        std::ostringstream s;
        s << std::fixed << std::setprecision( 3 ) << std::setfill( '0' ) <<
        "Time: " << m_currentTime << " s" << std::endl <<
             "Computation Time: " << m_computationTime << " ms" << std::endl <<
                "Current Stress: " << m_currentStress << std::endl <<
                    "AccumStress: " << m_stressSeconds << std::endl <<
                        "Active Units: " << m_nActiveHuman;
        return s.str();
    }

    double m_stressSeconds;
    double m_currentTime;
    double m_currentStress;
    size_t m_nActiveHuman;

    // simulation settings
    double m_maxSpeed;
    double m_maxAcceleration;

    int m_computationTime = 0;
};


#endif //CL_SIM_H
