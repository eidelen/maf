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
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QDoubleSpinBox>

Window::Window()
{
    setWindowTitle(tr("EidNN Lernfahrer"));

    GLWidget *openGL = new GLWidget(this);

    QPushButton* nextEpochBtn = new QPushButton("New Epoch");
    QPushButton* nextTrackBtn = new QPushButton("Next");
    QPushButton* saveBtn = new QPushButton("Save");
    QPushButton* loadBtn = new QPushButton("Load");
    QLabel* mutationRateLabel = new QLabel("Mutation Rate:");
    QDoubleSpinBox* mutationRateSpinBox = new QDoubleSpinBox();
    mutationRateSpinBox->setMinimum(0.0); mutationRateSpinBox->setMaximum(1.0);
    mutationRateSpinBox->setSingleStep(0.005); mutationRateSpinBox->setValue(0.05);


    QHBoxLayout* btnLayout = new QHBoxLayout();
    btnLayout->addWidget(nextEpochBtn);
    btnLayout->addWidget(nextTrackBtn);
    btnLayout->addWidget(saveBtn);
    btnLayout->addWidget(loadBtn);
    btnLayout->addWidget(mutationRateLabel);
    btnLayout->addWidget(mutationRateSpinBox);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(btnLayout);
    layout->addWidget(openGL);

    setLayout(layout);


    // connections

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, openGL, &GLWidget::animate);
    timer->start(10);

    connect(nextEpochBtn, SIGNAL (released()),openGL, SLOT (doNewEpoch()));
    connect(nextTrackBtn, SIGNAL (released()),openGL, SLOT (nextTrack()));
    connect(saveBtn, SIGNAL (released()),openGL, SLOT (save()));
    connect(loadBtn, SIGNAL (released()),openGL, SLOT (load()));
    connect(mutationRateSpinBox, SIGNAL(valueChanged(double)), openGL, SLOT( mutationRateChanged(double)));
}
