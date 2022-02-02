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

#ifndef DRAW_SCENE_H
#define DRAW_SCENE_H

#include <QPainter>
#include "simulation.h"
#include "nato_symbols.h"
#include "missile_station.h"
#include "proximity_sensor.h"
#include "plane.h"
#include "target.h"

class SimulationDrawer
{
public:
    SimulationDrawer(std::shared_ptr<Simulation> sim, Eigen::Vector2d center, double scale);
    ~SimulationDrawer();

    virtual void drawScene(QPainter& painter);
    virtual void drawMissileStation(QPainter& painter, MissileStation* station);
    virtual void drawProximitySensor(QPainter& painter, ProximitySensor* sensor);
    virtual void drawHostilePlane(QPainter& painter, HostilePlane* plane);
    virtual void drawMissile(QPainter& painter, Missile* missile);
    virtual void drawTarget(QPainter& painter, Target* target);

private:
    double sim2WidScale(double simLength);
    QPointF sim2WidTrans(const Eigen::Vector2d& simCoord);

    std::shared_ptr<Simulation> m_sim;
    std::shared_ptr<NatoSymbols> m_symbols;
    Eigen::Vector2d m_center;
    double m_scale;
    double m_symbolWidht = 60;
};

#endif //DRAW_SCENE_H
