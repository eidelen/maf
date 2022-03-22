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

#ifndef ENVIRONMENTINTERFACE_H
#define ENVIRONMENTINTERFACE_H

#include <memory>
#include <queue>
#include <Eigen/Dense>

#include "message.h"

/**
 * @brief The EnvironmentInterface class is an interface the
 * agents can use to access their einvironment. The agents should
 * have restricted view on the scene.
 */
class EnvironmentInterface
{

public:

    /**
     * Check if move is possible within environment. This function needs
     * to be overwritten in a derived Environment class.
     * @param origin Original position.
     * @param destination Target position.
     * @return Is move possible? When true, second result is usually the planned move
     * position. If false, the closest possible position can returnd.
     */
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const = 0;

    /**
     * @brief The Distance struct
     */
    struct Distance
    {
        double dist;
        unsigned int targetId;
        Eigen::Vector2d vect;
    };

    /**
     * @brief The compare distances function for priority queue
     */
    class CompareDistance
    {
    public:
        bool operator() (Distance a, Distance b)
        {
            return a.dist > b.dist;
        }
    };

    /**
     * DistanceQueue is able to maintain the lowest distance on the top.
     */
    using DistanceQueue = std::priority_queue<Distance, std::vector<Distance>, CompareDistance>;

    /**
     * Get all distances to all other agents for a given agent.
     * @param id Agent id.
     * @return Queue with closest agent first
     */
    virtual DistanceQueue getAgentDistancesToAllOtherAgents(unsigned int id) = 0;

    /**
     * MessageQueue contains the messages for a specific agent.
     */
    using MessageQueue = std::queue<std::shared_ptr<Message>>;

    /**
     * Get the messages for a specific agent. Note: Processing
     * this messages pops these from the queue permanently.
     * @param receiverAgendId Receiver agent id.
     * @return Message Queue
     */
    virtual MessageQueue& getMessages(unsigned int receiverAgendId) = 0;

    /**
     * Send a message.
     * @param aMessage A message.
     */
    virtual void sendMessage(std::shared_ptr<Message> aMessage) = 0;

    /**
     * Log a message to the std out.
     * @param logMsg log message.
     */
    virtual void log(const std::string& logMsg) = 0;

    /**
     * Compute the distance from a given position into a given direction
     * till the border of the environment is reached.
     * @param pos Position.
     * @param dir Direction.
     * @param stepSize Stepsize in which is checked if border rached.
     * @param maxDist At max distance, computation stops.
     * @return Distance to env border. If boarder cannot be rached within maxDist, maxDist is returned.
     */
    virtual double distanceToEnvironmentBorder(const Eigen::Vector2d& pos, const Eigen::Vector2d& dir,
                                               double stepSize, double maxDist) = 0;
};

#endif // ENVIRONMENTINTERFACE_H
