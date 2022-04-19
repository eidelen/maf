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

#ifndef SHAPES_H
#define SHAPES_H

#include <memory>
#include <Eigen/Dense>

enum ShapeType
{
    CircleShape,
    PolygonShape
};

/**
 * @brief The Shape class is an interface for all kind of 2d shapes used within an environment.
 */
class Shape
{
public:

    /**
     * @brief Quadrant constructor
     * @param id Id of area
     */
    Shape(unsigned int id);

    /**
     * Destructor.
     */
    virtual ~Shape();

    /**
     * Get the id of the qudrant
     */
    unsigned int id() const;

    /**
     * @brief Is a given coordinate within the shape area.
     * @param coordinate A coordinate.
     * @return True if within shape, otherwise false.
     */
    virtual bool isInShape(const Eigen::Vector2d& coordinate) = 0;

    /**
     * Get center of area
     */
    virtual Eigen::Vector2d center() const;

    /**
     * Return type of Shape.
     * @return Shape type.
     */
    virtual ShapeType type() const = 0;

protected:
    unsigned int m_id;
    Eigen::Vector2d m_center;
};

/**
 * @brief The Quadrant class is a rectangular area within the environment.
 */
class Quadrant: public Shape
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
     * Get upper left corner
     */
    Eigen::Vector2d upperLeft() const;

    /**
     * Get lower right corner
     */
    Eigen::Vector2d lowerRight() const;

    // inherit from Shape
    bool isInShape(const Eigen::Vector2d &coordinate) override;
    ShapeType type() const override;

private:
    Eigen::Vector2d m_upperLeft;
    Eigen::Vector2d m_lowerRight;

    // components used for fast comparisson
    double m_ux; double m_uy; double m_lx; double m_ly;
};

#endif // SHAPES_H
