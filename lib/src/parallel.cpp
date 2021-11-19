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

#include <thread>
#include <iostream>

#include "parallel.h"

std::shared_ptr<Parallel> Parallel::createParallel(size_t nThreads)
{
    return std::shared_ptr<Parallel>(new Parallel(nThreads));
}

Parallel::Parallel(size_t nThreads) : m_nbrThreads(nThreads)
{

}

Parallel::~Parallel()
{

}

void Parallel::addSimulation(std::shared_ptr<Simulation> aSimulation, double timeSteps, double simulationTime)
{
    m_simQueue.push({aSimulation, timeSteps, simulationTime});
}

void Parallel::run()
{
    std::vector<std::thread> workers;
    std::cout << "Run threads..." << std::endl;
    while(!m_simQueue.empty())
    {
        auto fr = m_simQueue.front();
        std::shared_ptr<Simulation> sim = std::get<0>(fr);
        double timeStep = std::get<1>(fr);
        double simDuration = std::get<2>(fr);

        m_simQueue.pop();






        std::cout << "Prepare sim " << sim->id() << std::endl;

        workers.push_back(std::thread([sim, timeStep, simDuration](){
            sim->initEnvironment();
            sim->initAgents();


            std::cout << "Init done " << sim->id() << std::endl;

            while(sim->getSimulationRunningTime() < simDuration)
            {
                sim->doTimeStep(timeStep);
            }

            std::cout << "Sim done " << sim->id() << std::endl;
        }));
    }

    std::for_each(workers.begin(), workers.end(), [](std::thread &t)
    {
        t.join();
    });

    std::cout << "All threads done..." << std::endl;
}
