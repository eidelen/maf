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

#ifndef PARALLEL_H
#define PARALLEL_H

#include <queue>
#include <thread>
#include <mutex>
#include "simulation.h"

/**
 * Runs simulations in parallel on multiple threads.
 */
class Parallel
{

public:

    using SimQueueElement = std::tuple<std::shared_ptr<Simulation>, double, double>;

    static std::shared_ptr<Parallel> createParallel(size_t nThreads);

    /**
     * Constructor.
     * @param nThreads How many threads to run in parallel.
     */
    Parallel(size_t nThreads);

    /**
     * Destructor
     */
    virtual ~Parallel();

    /**
     * Add a simulation
     * @param aSimulation Simulation
     * @param timeSteps Time steps in seconds.
     * @param simulationTime Overall simulation time in seconds.
     */
    void addSimulation( std::shared_ptr<Simulation> aSimulation, double timeSteps, double simulationTime );

    /**
     * Run all simulations
     */
    void run();

    /**
     * Main function of threads.
     */
    void doWork();

    /**
     * Get progress.
     * @return <number of finished simulations, number of queued simulations>
     */
    std::pair<double, double> getProgress() const;

private:
    std::queue<SimQueueElement> m_simQueue;
    size_t m_nbrThreads;
    std::vector<std::thread> m_threadPool;
    std::mutex m_queueMutex;
    std::atomic_size_t m_nbrOfFinishedSimulations;
};

#endif // PARALLEL_H
