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

#ifndef HELPERS_H
#define HELPERS_H

#include <Eigen/Dense>

#include "environment_interface.h"

class MafHlp
{
public:

    /**
     * Given a 2D vector, this method scales the vector if its magnitude exceeds a
     * certain maximum length.
     * @param in Vector
     * @param maxMagnitude Max. magnitude.
     * @return Scaled vector or same as input vector.
     */
    static Eigen::Vector2d correctVectorScale(const Eigen::Vector2d &in, double maxMagnitude)
    {
        double inLength = in.norm();
        if( inLength > 1.0e-10 && inLength > maxMagnitude ) // do not correct small vectors -> zero division
        {
            return (in / inLength) * maxMagnitude;
        }

        return in;
    }

    /**
     * Given a 2D vector, this method scales the vector so that
     * its length matches given magnitude.
     * @param in Vector
     * @param magnitude Wanted magnitude.
     * @return Scaled vector.
     */
    static Eigen::Vector2d adjustVectorScale(const Eigen::Vector2d &in, double magnitude)
    {
        double inLength = in.norm();
        if( inLength > 1.0e-10 ) // do not correct small vectors -> zero division
        {
            return (in / inLength) * magnitude;
        }

        return in;
    }

    /**
     * This function computes the acceleration required to stop an object of
     * a given velocity within a certain time. The resulting acceleration can,
     * however, not exceed the max. acceleration.
     * @param velocity Current velocity.
     * @param maxAcceleration Max. possible acceleration
     * @param time Time in which deacceleration happens.
     * @return Required acceleration.
     */
    static Eigen::Vector2d computeSlowDown(const Eigen::Vector2d& velocity, double maxAcceleration, double time)
    {
        double speed = velocity.norm();

        // Ignore slow objects
        if( speed < 1e-8 )
        {
            return Eigen::Vector2d(0.0, 0.0);
        }

        double reqAcc = speed / time;
        double chosenAcc = std::min(maxAcceleration, reqAcc);

        // unit acceleration opposite velocity and scaled
        return ((-velocity) / speed) * chosenAcc;
    }

    /**
     * Compute the average direction to all agents which are closer than a specified distance "obsDist".
     * A weighted averaging is chosen, so that closer agents matter more.
     * @param dists Distance queue
     * @param obsDist Observation distance
     * @return normalized average direction.
     */
    static std::pair<bool,Eigen::Vector2d> computeAvgWeightedDirectionToOtherAgents(EnvironmentInterface::DistanceQueue dists, double obsDist)
    {
        Eigen::Vector2d avgDir(0.0, 0.0);
        size_t cntAgents = 0;
        while(!dists.empty())
        {
            const auto& d = dists.top();
            if( d.dist < obsDist )
            {
                cntAgents++;

                // weigthed summation -> as more far the agent is, as less impact it has
                avgDir = avgDir + (d.vect * (1.0 / cntAgents));
                dists.pop();
            }
            else
            {
                // DistanceQueue is ordered -> next one is smaller as well
                break;
            }
        }

        if(cntAgents > 0)
            return {true, avgDir.normalized()};
        else
            return {false, avgDir};
    }

    /**
     * Returns specific Distance matching the given target agent id.
     * @param dists Distance Queue
     * @param targetAgentId Target agent id.
     * @return <found, Distance struct>
     */
    static std::pair<bool, EnvironmentInterface::Distance> getDistanceToAgent(EnvironmentInterface::DistanceQueue dists, unsigned int targetAgentId)
    {
        while(!dists.empty())
        {
            const auto& d = dists.top();
            if(d.targetId == targetAgentId )
            {
                return {true, d};
            }
            dists.pop();
        }

        return {false, EnvironmentInterface::Distance()};
    }


};

#endif // HELPERS_H
