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

#include "draw_scene.h"
#include "evaluation.h"

SimulationDrawer::SimulationDrawer(std::shared_ptr<Simulation> sim, Eigen::Vector2d center, double scale)
    : m_sim(sim), m_center(center), m_scale(scale)
{
    m_symbols = std::shared_ptr<NatoSymbols>(new NatoSymbols());
}

SimulationDrawer::~SimulationDrawer()
{

}

double SimulationDrawer::sim2WidScale(double simLength)
{
    return simLength * m_scale;
}

QPointF SimulationDrawer::sim2WidTrans(const Eigen::Vector2d& simCoord)
{
    Eigen::Vector2d widCoord = sim2WidTransE(simCoord);
    return toQPointF(widCoord);
}

Eigen::Vector2d SimulationDrawer::sim2WidTransE(const Eigen::Vector2d& simCoord)
{
    return (simCoord * sim2WidScale(1.0)) + m_center;
}

QPointF SimulationDrawer::toQPointF(const Eigen::Vector2d& coord)
{
    return QPointF(coord(0), coord(1));
}

void SimulationDrawer::drawScene(QPainter &painter)
{
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // draw agents
    auto agents = m_sim->getEnvironment()->getAgents();
    std::for_each(agents.begin(), agents.end(), [=, &painter](const auto& a) {

        if(a->type() == AgentType::EMissileStation)
        {
            drawMissileStation(painter, (MissileStation*)a.get());
        }
        else if(a->type() == AgentType::EMissile)
        {
            drawMissile(painter, (Missile*)a.get());
        }
        else if(a->type() == AgentType::EProxSensor)
        {
            drawProximitySensor(painter, (ProximitySensor*)a.get());
        }
        else if(a->type() == AgentType::EPlaneHostile)
        {
            drawHostilePlane(painter, (HostilePlane*)a.get());
        }
        else if(a->type() == AgentType::ETarget)
        {
            drawTarget(painter, (Target*)a.get());
        }
        else if(a->type() == AgentType::EHuman)
        {
            drawHuman(painter, (Human*)a.get());
        }
        else
        {
            painter.setBrush(Qt::blue);
            painter.drawEllipse(sim2WidTrans(a->getPosition()), m_symbolWidht/8, m_symbolWidht/8);
        }

    });

    // draw result text
    QFont font = painter.font();
    font.setPointSize(18);
    painter.setFont(font);
    painter.setPen(QColor(0,0,0));
    painter.drawText(QPoint(30,30), QString::fromStdString(m_sim->getEvaluation()->getResult()));
}

void SimulationDrawer::drawMissileStation(QPainter &painter, MissileStation* station)
{
    // draw rocket launcher symbol
    m_symbols->drawSymbolAt(NatoSymbols::RocketLauncher, m_symbolWidht, painter, sim2WidTrans(station->getPosition()));
}

void SimulationDrawer::drawProximitySensor(QPainter &painter, ProximitySensor *sensor)
{
    painter.setBrush(QColor(0,0,255, 40));
    painter.drawEllipse(sim2WidTrans(sensor->getPosition()),
                        sim2WidScale(sensor->range()), sim2WidScale(sensor->range()));
}

void SimulationDrawer::drawMissile(QPainter &painter, Missile *missile)
{
    if(missile->status() == Missile::Launched)
    {
        painter.setBrush(Qt::black);
        painter.drawEllipse(sim2WidTrans(missile->getPosition()), m_symbolWidht/10, m_symbolWidht/10);
    }
}

void SimulationDrawer::drawTarget(QPainter &painter, Target *target)
{
    QColor usedColor = target->getAgentsInSensorRange().size() > 0 ? QColor(255,0,0,120) : QColor(0,255,0,120);
    painter.setBrush(usedColor);
    painter.drawEllipse(sim2WidTrans(target->getPosition()),
                        sim2WidScale(target->range()), sim2WidScale(target->range()));
}

void SimulationDrawer::drawHuman(QPainter &painter, Human* human)
{
    // Color indicates stress
    double stress = human->getStressLevel();
    QColor agentColor( (int)(stress*255.0), 255-(int)(stress*255.0), 0.0 );
    painter.setBrush(agentColor);
    painter.drawEllipse(sim2WidTrans(human->getPosition()), m_symbolWidht/10, m_symbolWidht/10);

    // Acceleration direction
    Eigen::Vector2d posAgentScreen = sim2WidTransE(human->getPosition());
    Eigen::Vector2d lengthAccScreen = human->getAcceleration().normalized() * 10.0;
    painter.setPen(QPen(Qt::black, 1, Qt::SolidLine));
    painter.drawLine(toQPointF(posAgentScreen), QPointF(posAgentScreen(0)+lengthAccScreen(0), posAgentScreen(1)+lengthAccScreen(1)));
}

void SimulationDrawer::drawHostilePlane(QPainter &painter, HostilePlane* plane)
{
    if(plane->getEnabled())
    {
        m_symbols->drawSymbolAt(NatoSymbols::PlaneHostile, m_symbolWidht/2, painter, sim2WidTrans(plane->getPosition()));
    }
    else
    {
        m_symbols->drawSymbolAt(NatoSymbols::PlaneHostileDisabled, m_symbolWidht/2, painter, sim2WidTrans(plane->getPosition()));
    }
}
