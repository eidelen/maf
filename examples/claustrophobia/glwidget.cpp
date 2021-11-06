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

GLWidget::GLWidget(QWidget *parent): QOpenGLWidget(parent)
{
    setFixedSize(1200, 800);
    setAutoFillBackground(false);
}

void GLWidget::animate()
{
    update();
}

void GLWidget::paintEvent(QPaintEvent *event)
{
    // draw the simulation

    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(event->rect(), QBrush(QColor(64, 32, 64)));

    // draw text
    QFont font = painter.font() ;
    font.setPointSize(25);
    painter.setFont(font);
    painter.setPen(QColor(255,255,255));
    painter.drawText(QPoint(100,100), "Yeahh");

    painter.end();
}