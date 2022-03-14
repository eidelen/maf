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

#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <vector>
#include "agent.h"

class Objective
{

public:

    Objective(unsigned int id, std::vector<AgentWP> agents);

    /**
     * Destructor
     */
    virtual ~Objective();

    /**
     * Returns the id of the objective.
     * @return id
     */
    unsigned int id();

    /**
     * Get agents.
     * @return Agents.
     */
    std::vector<AgentWP> agents() const;

    /**
     * React towards objective. Has to be overwritten.
     * @param timeStep Time step in seconds.
     */
    virtual void react(double timeStep);

    /**
     * Objective is reached?
     * @return True if reached. Otherwise false.
     */
    virtual bool isDone() const;

protected:

    unsigned int m_id;
    std::vector<AgentWP> m_agents;
};

#endif // OBJECTIVE_H
