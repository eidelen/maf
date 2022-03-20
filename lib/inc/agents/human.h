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

    static std::shared_ptr<Human> createHuman(unsigned int id, double velocityLimit = 1.5,
                                              double maxAcceleration = 2.5, double obsDistance = 3.0,
                                              double reactionTime = 0.5);

    /**
     * Human like agent
     * @param id Id of the agent
     * @param maxSpeed Maximum speed the human can reach
     * @param maxAcceleration Maximum acceleration the human can apply
     * @param obsDistance Outside this distance, the human does not care.
     * @param reactionTime How long it takes till human performs action.
     */
    Human(unsigned int id, double velocityLimit, double maxAcceleration, double obsDistance, double reactionTime);

    /**
     * Destructor
     */
    virtual ~Human();

    /**
     * Disable this agent to react on nothing.
     * @param disable
     */
    void disableReacting(bool disable);

    /**
     * Get agent's stress level (0.0 -> 1.0)
     */
    double getStressLevel() const;


private:
    void computeStressLevel(EnvironmentInterface::DistanceQueue otherAgents);


public: // inherited from Agent
    std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMotion(double time) const override;
    void update(double time) override;
    AgentType type() const override;
    void performMove(double time) override;


protected:

    double m_obsDistance;
    bool m_disableReacting;
    double m_reactionTime;
    double m_timeSinceLastReaction;
    double m_stressLevel;

};



#endif // HUMAN_H
