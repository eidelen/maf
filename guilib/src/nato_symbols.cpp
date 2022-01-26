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

#include "nato_symbols.h"

NatoSymbols::NatoSymbols()
{
    Q_INIT_RESOURCE(resources);
    init();
}

NatoSymbols::~NatoSymbols()
{

}

QPixmap& NatoSymbols::getSymbol(NatoSymbols::Symbol sym)
{
    return m_symbols[sym];
}

QPixmap& NatoSymbols::getSymbolScaled(NatoSymbols::Symbol sym, int width)
{
    // check if already scaled
    std::pair<NatoSymbols::Symbol, int> key(sym, width);
    if(m_scaledSymbols.find(key) == m_scaledSymbols.end())
    {
        m_scaledSymbols[key] = getSymbol(sym).scaled(width, width, Qt::KeepAspectRatio);
    }

    return m_scaledSymbols[key];
}

void NatoSymbols::drawSymbolAt(NatoSymbols::Symbol sym, int width, QPainter& painter, QPointF pos)
{
    QPixmap& scaledImg = getSymbolScaled(sym, width);
    painter.drawPixmap( pos - QPointF(scaledImg.width()/2, scaledImg.height()/2),
                        scaledImg);
}

void NatoSymbols::init()
{
    QPixmap rocketLauncherSymbol("://symbols/rocketlauncher.png");
    m_symbols[NatoSymbols::RocketLauncher] = rocketLauncherSymbol;

    QPixmap planeHostile("://symbols/planehostile.png");
    m_symbols[NatoSymbols::PlaneHostile] = planeHostile;
}
