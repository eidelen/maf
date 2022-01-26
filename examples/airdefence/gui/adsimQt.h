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
#include "nato_symbols.h"


class HumanoidAgentQtSim
{
public:
    HumanoidAgentQtSim()
    {
        m_symbols = std::shared_ptr<NatoSymbols>(new NatoSymbols());
        restart();
    }

    virtual ~HumanoidAgentQtSim() {}

    void restart()
    {  
        m_sim = Simulation::createSimulation(4);
        m_sim->setAgentFactory(std::shared_ptr<CivilianAgentFactory>(new CivilianAgentFactory()));
        m_sim->setEnvironmentFactory(std::shared_ptr<CircEnvFactory>(new CircEnvFactory()));

        m_sim->initEnvironment();
        m_sim->initAgents();
    }

    void setTimeStep(double ts)
    {
        m_timeStep = ts;
    }

    void update()
    {
        m_sim->doTimeStep(m_timeStep);
    }

    QPointF sim2WidTrans(const Eigen::Vector2d& simCoord)
    {
        Eigen::Vector2d circOrigin(500.0, 400.0);
        Eigen::Vector2d widCoord = (simCoord * sim2WidScale(1.0)) + circOrigin;
        return QPointF(widCoord(0), widCoord(1));
    }

    double sim2WidScale(double simLength)
    {
        return simLength * 30.0;
    }

    void drawSim(QPainter& painter)
    {
        // draw circle
        double symbolWidht = 60;
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        painter.setBrush(Qt::white);
        painter.drawEllipse(sim2WidTrans(Eigen::Vector2d(0.0, 0.0)), sim2WidScale(10.0), sim2WidScale(10.0));



        // draw agents
        auto agents = m_sim->getEnvironment()->getAgents();
        std::for_each(agents.begin(), agents.end(), [=, &painter](const auto& a) {

            if(a->type() == AgentType::EMissileStation)
            {
                MissileStation* ms = (MissileStation*)a.get();

                // draw detection area
                painter.setBrush(QColor(0,0,255, 40));
                painter.drawEllipse(sim2WidTrans(a->getPosition()),
                                    sim2WidScale(ms->detectionRange()), sim2WidScale(ms->detectionRange()));

                // draw rocket launcher symbol
                m_symbols->drawSymbolAt(NatoSymbols::RocketLauncher, symbolWidht, painter, sim2WidTrans(ms->getPosition()));
            }
            else if(a->id() >= 10000 )
            {
                m_symbols->drawSymbolAt(NatoSymbols::PlaneHostile, symbolWidht/2, painter, sim2WidTrans(a->getPosition()));
            }
            else
            {

                painter.setBrush(Qt::black);
                painter.drawEllipse(sim2WidTrans(a->getPosition()), sim2WidScale(a->getRadius()), sim2WidScale(a->getRadius()));
            }

        });

        // draw text
        QFont font = painter.font();
        font.setPointSize(18);
        painter.setFont(font);
        painter.setPen(QColor(255,255,255));

        //painter.drawText(QPoint(30,30), QString::fromStdString(m_eval->getResult()));
    }

private:
    double m_timeStep;
    std::shared_ptr<Simulation> m_sim;
    std::shared_ptr<NatoSymbols> m_symbols;
};


#endif //CL_SIM_QT_H
