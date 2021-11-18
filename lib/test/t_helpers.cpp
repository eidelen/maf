#include <gtest/gtest.h>
#include "helpers.h"

TEST(Helpers, ScaleVectors)
{
    auto scaledV = MafHlp::correctVectorScale(Eigen::Vector2d(12.0, 0.0), 5.0);
    ASSERT_TRUE((scaledV - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Helpers, SlowDown)
{
    // zero speed object -> no acceleration
    ASSERT_TRUE((MafHlp::computeSlowDown(Eigen::Vector2d(0.0, 0.0), 10.0, 2.0) -
                 Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    // normal slow down -> -10 for 10 seconds
    ASSERT_TRUE((MafHlp::computeSlowDown(Eigen::Vector2d(100.0, 0.0), 12.0, 10.0) -
                 Eigen::Vector2d(-10.0, 0.0)).isMuchSmallerThan(0.0001));

    // limited slow down -> 10 for 10 seconds
    ASSERT_TRUE((MafHlp::computeSlowDown(Eigen::Vector2d(-200.0, 0.0), 10.0, 10.0) -
                 Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
}
