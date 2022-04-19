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

#include "objective.h"
#include "agent.h"
#include "message.h"
#include "environment.h"


Objective::Objective(unsigned int id,  int priority, AgentWP agent) :
    m_id(id), m_priority(priority), m_agent(agent), m_isStart(true)
{

}

Objective::~Objective()
{

}

unsigned int Objective::id() const
{
    return m_id;
}

int Objective::priority() const
{
    return m_priority;
}

void Objective::process(double timeStep)
{
    if(m_isStart)
    {
        startingAction();
        m_isStart = false;
    }

    react(timeStep);
}

void Objective::react(double /*timeStep*/)
{

}

bool Objective::isDone() const
{
    return true;
}

void Objective::setStartMessage(std::shared_ptr<Message> startMsg)
{
    m_startMessage = startMsg;
}

void Objective::setFinishMessage(std::shared_ptr<Message> finishMsg)
{
    m_finishMessage = finishMsg;
}

void Objective::startingAction()
{
    if(m_startMessage)
    {
        m_agent.lock()->getEnvironment().lock()->sendMessage(m_startMessage);
    }
}

void Objective::finishingAction()
{
    if(m_finishMessage)
    {
        m_agent.lock()->getEnvironment().lock()->sendMessage(m_finishMessage);
    }
}

//************** ObjectivePriorityQueue ********************//

void ObjectivePriorityQueue::popWhenDone()
{
    if(!empty() && top()->isDone() )
    {
        // also handle pre and post objective tasks

        top()->finishingAction();
        pop();
    }
}

void ObjectivePriorityQueue::processAndPopWhenDone(double timeStep)
{
    if( !empty() )
    {
        top()->react(timeStep);

        if(top()->isDone() )
        {
            // also handle pre and post objective tasks
            top()->finishingAction();
            pop();
        }
    }
}

void ObjectivePriorityQueue::deleteAllObjectives()
{
    while( !empty() )
        pop();
}


