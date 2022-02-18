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

#ifndef EVALUATION_H
#define EVALUATION_H

#include <memory>
#include <string>

class Simulation;

/**
 * @brief The Evaluation class can be used to evaluate a simulation. Needs
 * to be overwritten with specific implementation.
 */
class Evaluation
{

public:

    /**
     * Constructor
     */
    Evaluation();

    /**
     * Destructor
     */
    virtual ~Evaluation();

    /**
     * Evaluates the environment. Is called after each time step by
     * the simulation instant.
     * @param sim The environment.
     * @param timeStep Time step in seconds.
     */
    virtual void evaluate(std::shared_ptr<Simulation> sim, double timeStep);

    /**
     * Get the intermediate result as a string.
     * @return Result string.
     */
    virtual std::string getResult();

    /**
     * This function evaluates if the simulation
     * is finished. Needs to be overwritten.
     * @param sim The environment.
     * @return True if finished. False if still running.
     */
    virtual bool isSimulationFinished(std::shared_ptr<Simulation> sim);
};



#endif // EVALUATION_H
