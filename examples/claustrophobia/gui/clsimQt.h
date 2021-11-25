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
#include "clsimulation.h"


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
        m_stressSeconds = 0.0;

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
        painter.setBrush(Qt::white);
        painter.drawEllipse(sim2WidTrans(Eigen::Vector2d(0.0, 0.0)), sim2WidScale(10.0), sim2WidScale(10.0));

        // draw agents
        auto agents = m_sim->getEnvironment()->getAgents();
        double avgStress = 0.0;
        size_t cntAgents = 0;
        std::for_each(agents.begin(), agents.end(), [=, &painter, &avgStress, &cntAgents](const auto& a) {

            // cast from hell :)
            Human* h = (Human*)a.get();

            double stress = h->getStressLevel();

            avgStress += stress;
            cntAgents++;

            QColor agentColor( (int)(stress*255.0), 255-(int)(stress*255.0), 0.0 );
            painter.setBrush(agentColor);
            painter.drawEllipse(sim2WidTrans(h->getPosition()), sim2WidScale(h->getRadius()), sim2WidScale(h->getRadius()));

            painter.setPen(QPen(Qt::black, 1, Qt::SolidLine));
            painter.drawLine(sim2WidTrans(h->getPosition()), sim2WidTrans(h->getPosition() + (h->getAcceleration()/2.0)));

            if(h->type() == AgentType::ESoldier)
            {
                QFont sfont = painter.font();
                sfont.setPointSize(15);
                painter.setFont(sfont);
                painter.setPen(Qt::black);
                painter.drawText(sim2WidTrans(h->getPosition())+QPointF(-5,5), "S");

                QString stat;
                stat.sprintf("Time: %.3f, Stress: %.3f, AccStress: %.3f", m_sim->getSimulationRunningTime(), avgStress, m_stressSeconds);
                painter.drawText(QPoint(30,30), stat);
            }
        });

        avgStress = avgStress / cntAgents;
        m_stressSeconds += m_timeStep * avgStress;

        // draw text
        QFont font = painter.font();
        font.setPointSize(18);
        painter.setFont(font);
        painter.setPen(QColor(255,255,255));

        QString stat;
        stat.sprintf("Time: %.3f, Stress: %.3f, AccStress: %.3f", m_sim->getSimulationRunningTime(), avgStress, m_stressSeconds);
        painter.drawText(QPoint(30,30), stat);
    }

private:
    double m_timeStep;
    double m_stressSeconds;
    std::shared_ptr<Simulation> m_sim;
};


#endif //CL_SIM_QT_H