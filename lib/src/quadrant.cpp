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

#include "quadrant.h"

std::shared_ptr<Quadrant> Quadrant::createQuadrant(unsigned int id, const Eigen::Vector2d &upperLeft, const Eigen::Vector2d &lowerRight)
{
    return std::shared_ptr<Quadrant>(new Quadrant(id, upperLeft, lowerRight));
}

Quadrant::Quadrant(unsigned int id, const Eigen::Vector2d &upperLeft, const Eigen::Vector2d &lowerRight) :
    m_id(id), m_upperLeft(upperLeft), m_lowerRight(lowerRight)
{
    m_center = (m_upperLeft + m_lowerRight) / 2.0;
    m_ux = m_upperLeft.x();
    m_uy = m_upperLeft.y();
    m_lx = m_lowerRight.x();
    m_ly = m_lowerRight.y();
}

Quadrant::~Quadrant()
{

}

unsigned int Quadrant::id() const
{
    return m_id;
}

Eigen::Vector2d Quadrant::upperLeft() const
{
    return m_upperLeft;
}

Eigen::Vector2d Quadrant::lowerRight() const
{
    return m_lowerRight;
}

Eigen::Vector2d Quadrant::center() const
{
    return m_center;
}

bool Quadrant::isInQuadrant(const Eigen::Vector2d &coordinate)
{
    double cx = coordinate.x();
    double cy = coordinate.y();
    return !((cx < m_ux || cx > m_lx) || (cy < m_uy || cy > m_ly));
}
