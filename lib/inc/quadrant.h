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

#ifndef QUADRANT_H
#define QUADRANT_H

#include <memory>
#include <Eigen/Dense>

/**
 * @brief The Quadrant class is a rectangular area within the environment.
 */
class Quadrant
{

public:

    /**
     * @brief createQuadrant
     * @param id Id of area
     * @param upperLeft Upper left coordinate
     * @param lowerRight Lower right coordinate
     * @return pointer to quadrant
     */
    static std::shared_ptr<Quadrant> createQuadrant(unsigned int id, const Eigen::Vector2d& upperLeft,
                                                    const Eigen::Vector2d& lowerRight);

    /**
     * @brief Quadrant constructor
     * @param id Id of area
     * @param upperLeft Upper left coordinate
     * @param lowerRight Lower right coordinate
     */
    Quadrant(unsigned int id, const Eigen::Vector2d& upperLeft,
             const Eigen::Vector2d& lowerRight);

    /**
     * Destructor.
     */
    virtual ~Quadrant();

    /**
     * Get the id of the qudrant
     */
    unsigned int id() const;

    /**
     * Get upper left corner
     */
    Eigen::Vector2d upperLeft() const;

    /**
     * Get lower right corner
     */
    Eigen::Vector2d lowerRight() const;

    /**
     * Get center of area
     */
    Eigen::Vector2d center() const;

    /**
     * @brief Is a given coordinate within the area.
     * @param coordinate A coordinate.
     * @return True if within quadrant, otherwise false.
     */
    bool isInQuadrant(const Eigen::Vector2d& coordinate);

private:
    unsigned int m_id;
    Eigen::Vector2d m_upperLeft;
    Eigen::Vector2d m_lowerRight;
    Eigen::Vector2d m_center;

    // components used for fast comparisson
    double m_ux; double m_uy; double m_lx; double m_ly;
};

#endif // QUADRANT_H