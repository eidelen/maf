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

#ifndef PLANE_H
#define PLANE_H


#include "agent.h"


/**
 * @brief This class represents a basic plane agent.
 */
class Plane: public Agent
{

public:

    /**
     * Plane like agent
     * @param id Id of the agent
     */
    Plane(unsigned int id);

    /**
     * Destructor
     */
    virtual ~Plane();

public: // inherited from Agent
    AgentType type() const override;
};

class HostilePlane: public Plane
{
public:
    /**
     * Hostile plane
     * @param id Id of the agent
     */
    HostilePlane(unsigned int id);

    /**
     * Destructor
     */
    virtual ~HostilePlane();

public: // inherited from Agent
    AgentType type() const override;

};

#endif // PLANE_H
