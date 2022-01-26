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

#ifndef CL_SIM_QT_H
#define CL_SIM_QT_H

#include <QPainter>
#include "agent.h"
#include "airdefencesim.h"
#include "simulation.h"
#include "draw_scene.h"


class HumanoidAgentQtSim
{
public:
    HumanoidAgentQtSim()
    {
        restart();
    }

    virtual ~HumanoidAgentQtSim() {}

    void restart()
    {  
        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<AirdefenceAgentFactory>(new AirdefenceAgentFactory()));
        m_sim->setEnvironmentFactory(std::shared_ptr<PlaneEnvFactory>(new PlaneEnvFactory()));

        m_sim->initEnvironment();
        m_sim->initAgents();

        m_drawer = std::shared_ptr<EnvironmentDrawer>(new EnvironmentDrawer(m_sim->getEnvironment(),
                                                                            Eigen::Vector2d(500.0, 400.0), 30.0));
    }

    void setTimeStep(double ts)
    {
        m_timeStep = ts;
    }

    void update()
    {
        m_sim->doTimeStep(m_timeStep);
    }

    void drawSim(QPainter& painter)
    {
        m_drawer->drawScene(painter);
    }

private:
    double m_timeStep;
    std::shared_ptr<Simulation> m_sim;
    std::shared_ptr<EnvironmentDrawer> m_drawer;
};


#endif //CL_SIM_QT_H
