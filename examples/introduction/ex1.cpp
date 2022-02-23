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

#include <chrono>
#include <iostream>

#include "simulation.h"
#include "agent.h"

int main(int argc, char *argv[])
{
    // create an empty environment
    auto env = Environment::createEnvironment(0);

    // create two accelerating agents
    auto flashGordon = Agent::createAgent(1);
    flashGordon->setPosition(Eigen::Vector2d(0.0, 0.0));
    flashGordon->setAcceleration(Eigen::Vector2d(1.0, 0.0));
    flashGordon->setEnvironment(env);

    auto batman = Agent::createAgent(2);
    batman->setPosition(Eigen::Vector2d(0.0, 0.0));
    batman->setAcceleration(Eigen::Vector2d(0.8, 0.2));
    batman->setEnvironment(env);

    // add agents to environmet
    env->addAgent(flashGordon);
    env->addAgent(batman);

    // start simulation - stop when any agent reaches a speed > 5 m/s
    double timeStep = 0.5;
    double runTime = 0.0;
    while( true )
    {
        // print positions of all agents
        std::cout << "--------- " << runTime << " s ------------" << std::endl;
        auto allAgents = env->getAgents();
        std::for_each(allAgents.begin(), allAgents.end(), [=](std::shared_ptr<Agent> a)
        {
            std::cout << "Agent " << a->id() <<
                         " , Position (" << a->getPosition().transpose() <<
                            ") , Speed (" << a->getVelocity().transpose()<< ")" << std::endl;
        });

        // update simulation
        env->update(timeStep);
        runTime += timeStep;

        // check for exit
        if(std::any_of(allAgents.begin(), allAgents.end(), [](std::shared_ptr<Agent> a){
            return a->getVelocity().norm() > 5.0;
        }))
            break;
    }
}






/*
    for demo
    auto m = std::shared_ptr<Missile>(new Missile(3));
    m->setVelocityLimit(4.0);
    m->setPosition(Eigen::Vector2d(-7.0, -7.0));
    m->setEnvironment(env);
    m->fire(1);

    */
