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

#include "window.h"
#include "glwidget.h"

#include <QLayout>
#include <QTimer>
#include <QPushButton>

Window::Window()
{
    setWindowTitle(tr("maf"));

    GLWidget *openGL = new GLWidget(this);

    // create simulation and pass and instance to GL_Widget
    double timeStep = 0.05; //25ms
    m_hSim = std::shared_ptr<HumanoidAgentQtSim>(new HumanoidAgentQtSim());
    m_hSim->setTimeStep(timeStep);
    openGL->setQtSimulation(m_hSim);

    QPushButton* resetBtn = new QPushButton("Restart");

    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addWidget(resetBtn);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(btnLayout);
    layout->addWidget(openGL);

    setLayout(layout);


    // connections

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, openGL, &GLWidget::animate);
    timer->start(static_cast<int>(timeStep*1000));

    connect(resetBtn, SIGNAL (released()),this, SLOT(resetSimulation()));
}

void Window::resetSimulation()
{
    m_hSim->restart();
}
