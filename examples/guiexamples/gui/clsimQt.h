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
#include "simQt.h"
#include "clsimulation.h"
#include "draw_scene.h"


class HumanoidAgentQtSim: public SimQt
{
public:
    HumanoidAgentQtSim(): SimQt()
    {
        restart();
    }

    virtual ~HumanoidAgentQtSim() {}

    void restart() override
    {
        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<CivilianAgentFactory>(new CivilianAgentFactory()));
        m_sim->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

        m_eval = std::shared_ptr<StressAccumulatorEvaluation>(new StressAccumulatorEvaluation(0.0, 0.0));
        m_sim->setEvaluation(m_eval);

        m_sim->initEnvironment();
        m_sim->initAgents();

        m_drawer = std::shared_ptr<SimulationDrawer>(new SimulationDrawer(m_sim, Eigen::Vector2d(500.0, 400.0), 30.0));
    }

    void drawSim(QPainter& painter) override
    {
        // draw background / environment
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(Qt::white);
        painter.drawEllipse(m_drawer->sim2WidTrans(Eigen::Vector2d(0.0, 0.0)), m_drawer->sim2WidScale(10.0), m_drawer->sim2WidScale(10.0));

        m_drawer->drawScene(painter);
    }

private:
    std::shared_ptr<StressAccumulatorEvaluation> m_eval;
};


#endif //CL_SIM_QT_H
