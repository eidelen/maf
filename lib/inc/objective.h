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
#include <memory>
#include <queue>

class Agent;
class Message;

/**
 * @brief The Objective class represents an interface which
 * allows to implement agent behaviours or actions.
 */
class Objective
{

public:

    /**
     * Constructor for Objective.
     * @param id Objective id.
     * @param priority Objectives priority, where low is highest priority.
     * @param agent To which agent objective belongs.
     */
    Objective(unsigned int id, int priority, std::weak_ptr<Agent> agent);

    /**
     * Destructor
     */
    virtual ~Objective();

    /**
     * Returns the id of the objective.
     * @return id
     */
    unsigned int id() const;

    /**
     * Get the priority of the objective.
     * @return
     */
    int priority() const;

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

    /**
     * This message is sent when the objective gets active.
     * @param startMsg Start message.
     */
    void setStartMessage(std::shared_ptr<Message> startMsg);

    /**
     * This message is sent when the objective is done.
     * @param finishMsg Finishing message.
     */
    void setFinishMessage(std::shared_ptr<Message> finishMsg);

    /**
     * Perform an action when the objective starts.
     * Note: Default implementation sends starting message.
     */
    virtual void startingAction();

    /**
     * Perform an action when the objective ends (isDone = true).
     * Note: Default implementation sends finishing message.
     */
    virtual void finishingAction();

    /**
     * Process the objective
     * @param timeStep Time step in seconds.
     */
    void process(double timeStep);


protected:

    unsigned int m_id;
    int m_priority;
    std::weak_ptr<Agent> m_agent;
    std::shared_ptr<Message> m_startMessage;
    std::shared_ptr<Message> m_finishMessage;
    bool m_isStart;
};


using ObjectiveSP = std::shared_ptr<Objective>;


/**
 * Objective priority queue.
 */
class CompareObjectivePrios
{
public:
    bool operator() (ObjectiveSP a, ObjectiveSP b)
    {
        return a->priority() > b->priority();
    }
};

class ObjectivePriorityQueue: public std::priority_queue<ObjectiveSP, std::vector<ObjectiveSP>, CompareObjectivePrios>
{
public:

    /**
     * Pop top objective when done.
     */
    void popWhenDone();
    
    /**
     * Process top objective and remove when done.
     */
    void processAndPopWhenDone(double timeStep);
};

#endif // OBJECTIVE_H
