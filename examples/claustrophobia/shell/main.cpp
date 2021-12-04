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
#include <vector>

#include "parallel.h"
#include "clsimulation.h"

int main(int argc, char *argv[])
{
    using namespace std::chrono_literals;
    auto start = std::chrono::high_resolution_clock::now();

    auto p = Parallel::createParallel(std::thread::hardware_concurrency());

    size_t nSims = 0;
    double tStep = 0.2;
    double simDur = 30.0;

    std::vector<std::shared_ptr<StressAccumulatorEvaluation>> results;

    for(unsigned int a = 0; a < 100; a++)
    {
        for(unsigned int v = 0; v < 100; v++ )
        {
            double maxSpeed = 0.1+0.1*v;
            double maxAcceleration = 0.1+0.1*a;

            auto sim = Simulation::createSimulation(nSims++);

            sim->setAgentFactory(std::shared_ptr<CivilianAgentFactory>(new CivilianAgentFactory(maxSpeed, maxAcceleration)));
            sim->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

            auto eval = std::shared_ptr<StressAccumulatorEvaluation>(new StressAccumulatorEvaluation(maxSpeed, maxAcceleration));
            sim->setEvaluation(eval);
            results.push_back(eval);

            p->addSimulation(sim, tStep, simDur);
        }
    }

    p->run();

    while(true)
    {
        auto[done, queued] = p->getProgress();
        std::cout << "[" << done << ", " << queued << "]" << std::endl;

        if(done == nSims)
            break;

        std::this_thread::sleep_for(10000ms);
    }

    auto end = std::chrono::high_resolution_clock::now();

    // print results
    std::sort(results.begin(), results.end(), [](std::shared_ptr<StressAccumulatorEvaluation> a,
                std::shared_ptr<StressAccumulatorEvaluation> b)
    {
        return a->m_stressSeconds < b->m_stressSeconds;
    });

    std::cout << "max v, max a, stress seconds, sim time" << std::endl;
    std::for_each(results.begin(), results.begin()+10, [](std::shared_ptr<StressAccumulatorEvaluation> r)
    {
        std::cout << r->m_maxSpeed << ", " << r->m_maxAcceleration << ", "
                  << r->m_stressSeconds << ", " << r->m_currentTime << std::endl;
    });

    std::chrono::duration<double, std::milli> elapsed = end-start;
    std::cout << "Waited " << elapsed.count() << " ms" << std::endl;
    std::cout << "Time per step: " << elapsed.count() / (nSims * (simDur/tStep)) << " ms" << std::endl;

    return 0;
}
