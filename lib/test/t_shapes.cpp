
#include <gtest/gtest.h>
#include "shapes.h"

TEST(Quadrant, Basics)
{
    auto q = Quadrant::createQuadrant(99, Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(5.0, 4.0));
    ASSERT_EQ(q->id(), 99);
    ASSERT_TRUE(q->type() == ShapeType::PolygonShape);
    ASSERT_TRUE((q->upperLeft() - Eigen::Vector2d(2.0, 3.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((q->lowerRight() - Eigen::Vector2d(5.0, 4.0)).isMuchSmallerThan(0.0001));
}

TEST(Quadrant, Center)
{
    auto q = Quadrant::createQuadrant(99, Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(5.0, 4.0));

    ASSERT_EQ(q->id(), 99);
    ASSERT_TRUE((q->center() - Eigen::Vector2d(3.5, 3.5)).isMuchSmallerThan(0.0001));
}

TEST(Quadrant, IsWithin)
{
    auto q = Quadrant::createQuadrant(99, Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(5.0, 4.0));

    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(0.0, 0.0)));
    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(3.0, 0.0)));
    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(3.0, 2.99)));
    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(3.0, 4.0011)));
    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(1.99, 3.5)));
    ASSERT_FALSE(q->isInShape(Eigen::Vector2d(5.001, 3.5)));

    ASSERT_TRUE(q->isInShape(Eigen::Vector2d(2.0, 3.0)));
    ASSERT_TRUE(q->isInShape(Eigen::Vector2d(5.0, 4.0)));
    ASSERT_TRUE(q->isInShape(Eigen::Vector2d(5.0, 3.0)));
    ASSERT_TRUE(q->isInShape(Eigen::Vector2d(2.0, 4.0)));
    ASSERT_TRUE(q->isInShape(Eigen::Vector2d(3.5, 3.5)));
}

//***************************************************//

TEST(Circle, Basics)
{
    auto c = std::shared_ptr<Circle>(new Circle(3, Eigen::Vector2d(2.0, 3.0), 5.0));
    ASSERT_EQ(c->id(), 3);
    ASSERT_TRUE(c->type() == ShapeType::CircleShape);
    ASSERT_NEAR(c->radius(), 5.0, 0.0001);
}

TEST(Circle, Center)
{
    auto c = std::shared_ptr<Circle>(new Circle(3, Eigen::Vector2d(2.0, 3.0), 5.0));
    ASSERT_TRUE((c->center() - Eigen::Vector2d(2.0, 3.0)).isMuchSmallerThan(0.0001));
}

TEST(Circle, IsWithin)
{
    auto c = std::shared_ptr<Circle>(new Circle(3, Eigen::Vector2d(2.0, 3.0), 2.0));

    ASSERT_FALSE(c->isInShape(Eigen::Vector2d(0.0, 0.0)));
    ASSERT_FALSE(c->isInShape(Eigen::Vector2d(2.0, 0.9999)));
    ASSERT_FALSE(c->isInShape(Eigen::Vector2d(-0.1, 3.0)));

    ASSERT_TRUE(c->isInShape(Eigen::Vector2d(2.0, 1.1)));
    ASSERT_TRUE(c->isInShape(Eigen::Vector2d(0.1, 3.0)));
}

