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

#include "glwidget.h"

#include <QPainter>
#include <QTimer>
#include <QPaintEvent>
#include <QRgb>

#include <iostream>

GLWidget::GLWidget(QWidget *parent): QOpenGLWidget(parent)
{
    setFixedSize(1500, 1000);
}

void GLWidget::setQtSimulation(std::shared_ptr<SimQt> sim)
{
    m_sim = sim;
}

void GLWidget::animate()
{
    m_sim->update();
    update();
}

void GLWidget::paintEvent(QPaintEvent *event)
{
    // draw the simulation

    QPainter painter;
    painter.begin(this);

    painter.setBrush(QColor(150,150,150));
    painter.drawRect(event->rect());

    // draw scene
    m_sim->drawSim(painter);

    painter.end();
}
