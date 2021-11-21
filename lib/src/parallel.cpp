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

#include <iostream>
#include <algorithm>

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
    for (std::thread& t : m_threadPool)
    {
        t.join();
    }
}

void Parallel::addSimulation(std::shared_ptr<Simulation> aSimulation, double timeSteps, double simulationTime)
{
    m_simQueue.push({aSimulation, timeSteps, simulationTime});
}

void Parallel::doWork()
{
    while (true)
    {
        SimQueueElement sq;
        bool gotSim = false;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);

            if( m_simQueue.empty() )
            {
                std::cout << "Nothing left to do" << std::endl;
                return;
            }

            sq = m_simQueue.front();
            gotSim = true;
            m_simQueue.pop();

            // lock is unlocked when leavin scope
        }

        if(gotSim)
        {
            // run the simulation
            std::shared_ptr<Simulation> sim = std::get<0>(sq);
            std::cout << "Run simulation " << sim->id() << std::endl;;
        }

    }
};

void Parallel::run()
{
    for(size_t k = 0; k < m_nbrThreads; k++)
    {
        std::cout << "Create Thread " << k << std::endl;
        m_threadPool.push_back(std::thread([this] { this->doWork(); } ));
    }
}
