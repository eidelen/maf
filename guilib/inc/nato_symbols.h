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

#ifndef NATO_SYMBOLS_H
#define NATO_SYMBOLS_H

#include <QPainter>
#include <QPixmap>
#include <map>


/**
 * @brief The NatoSymbols class encapsulates the NATO
 * standard symbols.
 */
class NatoSymbols
{
public:
    enum Symbol{
        RocketLauncher
    };

    NatoSymbols();
    virtual ~NatoSymbols();

    /**
     * Get the symbol as QPixmap.
     * @param sym Symbol identifier.
     * @return Image as QPixmap.
     */
    QPixmap& getSymbol(Symbol sym);

    /**
     * Get the symbol as scaled
     * QPixmap
     * @param sym Symbol identifier.
     * @param width pixel width
     * @return Image as QPixmap
     */
    QPixmap getSymbolScaled(Symbol sym, int width);

    /**
     * Draw symbol at position with specific widht.
     * @param sym Symobol id.
     * @param width Image width.
     * @param painter Widget painter.
     * @param pos symbol position.
     */
    void drawSymbolAt(Symbol sym, int width, QPainter& painter, QPointF pos);

private:
    void init();

private:
    std::map<Symbol, QPixmap> m_symbols;

};


#endif //NatoSymbols
