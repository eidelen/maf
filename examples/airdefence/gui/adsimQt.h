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
#include "agent.h"
#include "airdefencesim.h"
#include "simulation.h"
#include "draw_scene.h"


class AirDefenceQtSim: public SimQt
{
public:
    AirDefenceQtSim(): SimQt()
    {
        restart();
    }

    virtual ~AirDefenceQtSim() {}

    void restart() override
    {
        // load background
        QPixmap swissmap("://symbols/swissmap.png");
        m_background = swissmap.scaled(1500, 1500, Qt::KeepAspectRatio, Qt::SmoothTransformation);

        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<AirdefenceAgentFactory>(new AirdefenceAgentFactory()));
        m_sim->setEnvironmentFactory(std::shared_ptr<PlaneEnvFactory>(new PlaneEnvFactory()));

        m_eval = std::shared_ptr<ReachEvaluation>(new ReachEvaluation());
        m_sim->setEvaluation(m_eval);

        m_sim->initEnvironment();
        m_sim->initAgents();

        m_drawer = std::shared_ptr<SimulationDrawer>(new SimulationDrawer(m_sim, Eigen::Vector2d(504.0, 423.0), 0.004));
    }

    void drawSim(QPainter& painter) override
    {
        // draw background
        painter.setRenderHint(QPainter::Antialiasing);
        painter.drawPixmap(QPointF(0.0, 0.0), m_background);

        m_drawer->drawScene(painter);
    }

private:
    std::shared_ptr<SimulationDrawer> m_drawer;
    QPixmap m_background;
    std::shared_ptr<ReachEvaluation> m_eval;
};


#endif //CL_SIM_QT_H
