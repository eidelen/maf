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
#include "missile_station.h"
#include "plane.h"

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
    double symbolWidht = 60;
    painter.setRenderHint(QPainter::SmoothPixmapTransform);

    // draw agents
    auto agents = m_env->getAgents();
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
        else if(a->type() == AgentType::EPlane)
        {
            m_symbols->drawSymbolAt(NatoSymbols::PlaneHostile, symbolWidht/2, painter, sim2WidTrans(a->getPosition()));
        }
        else
        {
            painter.setBrush(Qt::black);
            painter.drawEllipse(sim2WidTrans(a->getPosition()), sim2WidScale(a->getRadius()), sim2WidScale(a->getRadius()));
        }

    });
}
