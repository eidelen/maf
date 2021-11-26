
#include <gtest/gtest.h>
#include "quadrant.h"

TEST(Quadrant, Basics)
{
    auto q = Quadrant::createQuadrant(99, Eigen::Vector2d(2.0, 3.0), Eigen::Vector2d(5.0, 4.0));
    ASSERT_EQ(q->id(), 99);
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

    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(0.0, 0.0)));
    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(3.0, 0.0)));
    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(3.0, 2.99)));
    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(3.0, 4.0011)));
    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(1.99, 3.5)));
    ASSERT_FALSE(q->isInQuadrant(Eigen::Vector2d(5.001, 3.5)));

    ASSERT_TRUE(q->isInQuadrant(Eigen::Vector2d(2.0, 3.0)));
    ASSERT_TRUE(q->isInQuadrant(Eigen::Vector2d(5.0, 4.0)));
    ASSERT_TRUE(q->isInQuadrant(Eigen::Vector2d(5.0, 3.0)));
    ASSERT_TRUE(q->isInQuadrant(Eigen::Vector2d(2.0, 4.0)));
    ASSERT_TRUE(q->isInQuadrant(Eigen::Vector2d(3.5, 3.5)));
}

