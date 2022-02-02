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

EnvironmentDrawer::EnvironmentDrawer(std::shared_ptr<Environment> env, Eigen::Vector2d center, double scale)
    : m_env(env), m_center(center), m_scale(scale)
{
    m_symbols = std::shared_ptr<NatoSymbols>(new NatoSymbols());
}

EnvironmentDrawer::~EnvironmentDrawer()
{

}

double EnvironmentDrawer::sim2WidScale(double simLength)
{
    return simLength * m_scale;
}

QPointF EnvironmentDrawer::sim2WidTrans(const Eigen::Vector2d& simCoord)
{
    Eigen::Vector2d widCoord = (simCoord * sim2WidScale(1.0)) + m_center;
    return QPointF(widCoord(0), widCoord(1));
}

void EnvironmentDrawer::drawScene(QPainter &painter)
{
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // draw agents
    auto agents = m_env->getAgents();
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
        else
        {
            painter.setBrush(Qt::blue);
            painter.drawEllipse(sim2WidTrans(a->getPosition()), m_symbolWidht/8, m_symbolWidht/8);
        }

    });
}

void EnvironmentDrawer::drawMissileStation(QPainter &painter, MissileStation* station)
{
    // draw rocket launcher symbol
    m_symbols->drawSymbolAt(NatoSymbols::RocketLauncher, m_symbolWidht, painter, sim2WidTrans(station->getPosition()));
}

void EnvironmentDrawer::drawProximitySensor(QPainter &painter, ProximitySensor *sensor)
{
    painter.setBrush(QColor(0,0,255, 40));
    painter.drawEllipse(sim2WidTrans(sensor->getPosition()),
                        sim2WidScale(sensor->range()), sim2WidScale(sensor->range()));
}

void EnvironmentDrawer::drawMissile(QPainter &painter, Missile *missile)
{
    if(missile->status() == Missile::Launched)
    {
        painter.setBrush(Qt::black);
        painter.drawEllipse(sim2WidTrans(missile->getPosition()), m_symbolWidht/10, m_symbolWidht/10);
    }
}

void EnvironmentDrawer::drawTarget(QPainter &painter, Target *target)
{
    QColor usedColor = target->getAgentsInSensorRange().size() > 0 ? QColor(255,0,0,120) : QColor(0,255,0,120);
    painter.setBrush(usedColor);
    painter.drawEllipse(sim2WidTrans(target->getPosition()),
                        sim2WidScale(target->range()), sim2WidScale(target->range()));
}

void EnvironmentDrawer::drawHostilePlane(QPainter &painter, HostilePlane* plane)
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