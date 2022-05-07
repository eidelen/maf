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

#include <QLayout>
#include <QTimer>
#include <QPushButton>

#include "adsimQt.h"
#include "clsimQt.h"
#include "simQt.h"

Window::Window()
{
    setWindowTitle(tr("Multi Agent Framework"));

    m_OpenGL = new GLWidget(this);

    m_messageWindow = new MessageWindow(this);

    // Default sim
    startAirDefenceSim();

    m_ffSlider = new QSlider(Qt::Orientation::Horizontal);
    m_ffSlider->setMinimum(1); m_ffSlider->setMaximum(100); m_ffSlider->setValue(40);

    QPushButton* restartBtn = new QPushButton("Restart");
    QPushButton* airDefBtn = new QPushButton("Air Defence");
    QPushButton* clBtn = new QPushButton("Claustrophobia");
    m_dbgCB = new QCheckBox("Dbg");
    m_showMessagesWindow = new QCheckBox("Msg");

    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addWidget(m_ffSlider);
    btnLayout->addWidget(airDefBtn);
    btnLayout->addWidget(clBtn);
    btnLayout->addWidget(restartBtn);
    btnLayout->addWidget(m_dbgCB);
    btnLayout->addWidget(m_showMessagesWindow);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(btnLayout);
    layout->addWidget(m_OpenGL);

    setLayout(layout);


    // connections

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, m_OpenGL, &GLWidget::animate);

    connect(restartBtn, SIGNAL (released()),this, SLOT(resetSimulation()));
    connect(airDefBtn, SIGNAL (released()),this, SLOT(startAirDefenceSim()));
    connect(clBtn, SIGNAL (released()),this, SLOT(startCLSim()));
    connect(m_ffSlider, SIGNAL (valueChanged(int)),this, SLOT(adjustFastForwardSpeed()));
    connect(m_dbgCB, SIGNAL(stateChanged(int)), this, SLOT(dbgCBChanged()));
    connect(m_showMessagesWindow, SIGNAL(stateChanged(int)), this, SLOT(messageWinChanged()));


    adjustFastForwardSpeed();

    // adapt gui to presets
    m_dbgCB->setChecked(m_Sim->getDrawer()->getDrawDebugInfo());
    m_showMessagesWindow->setChecked(false);
}

void Window::resetSimulation()
{
    m_Sim->restart();
}

void Window::startAirDefenceSim()
{
    m_timeStep = 1.0; //1s
    m_Sim = std::shared_ptr<AirDefenceQtSim>(new AirDefenceQtSim());
    m_Sim->setTimeStep(m_timeStep);
    m_OpenGL->setQtSimulation(m_Sim);
}

void Window::startCLSim()
{
    m_timeStep = 0.05; //50ms
    m_Sim = std::shared_ptr<HumanoidAgentQtSim>(new HumanoidAgentQtSim());
    m_Sim->setTimeStep(m_timeStep);
    m_OpenGL->setQtSimulation(m_Sim);
}

void Window::adjustFastForwardSpeed()
{
    m_timer->stop();

    double ffVal = (double)m_ffSlider->value();
    m_timer->start(static_cast<int>((m_timeStep*1000.0)/ffVal));
}

void Window::dbgCBChanged()
{
    m_Sim->getDrawer()->setDrawDebugInfo(m_dbgCB->isChecked());
}

void Window::messageWinChanged()
{
    if(m_showMessagesWindow->isChecked())
    {
        m_messageWindow->show();
        m_messageWindow->setFocus();
    }
    else
    {
        m_messageWindow->hide();
    }
}
