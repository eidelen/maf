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

TEST(Helpers, WeightedAgentDir)
{
    EnvironmentInterface::DistanceQueue q;
    q.push({1.0, 2, Eigen::Vector2d(1.0, 0.0)});
    q.push({2.0, 3, Eigen::Vector2d(0.0, 2.0)});

    // consider all
    auto[ok1, avg1] = MafHlp::computeAvgWeightedDirectionToOtherAgents(q, 3.0);

    ASSERT_TRUE(ok1);
    ASSERT_TRUE((avg1 - Eigen::Vector2d(1.0 / sqrt(2.0), 1.0 / sqrt(2.0))).isMuchSmallerThan(0.0001));

    // consider one
    auto[ok2, avg2] = MafHlp::computeAvgWeightedDirectionToOtherAgents(q, 1.5);

    ASSERT_TRUE(ok2);
    ASSERT_TRUE((avg2 - Eigen::Vector2d(1.0, 0.0)).isMuchSmallerThan(0.0001));

    // consider No
    auto[ok3, avg3] = MafHlp::computeAvgWeightedDirectionToOtherAgents(q, 0.5);
    ASSERT_FALSE(ok3);

    // epmty queue
    EnvironmentInterface::DistanceQueue z;
    auto[ok4, avg4] = MafHlp::computeAvgWeightedDirectionToOtherAgents(z, 3.0);
    ASSERT_FALSE(ok4);
}
