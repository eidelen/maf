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

TEST(Helpers, ReadSpecifAgentDist)
{
    EnvironmentInterface::DistanceQueue q;
    q.push({1.0, 2, Eigen::Vector2d(1.0, 0.0)});
    q.push({2.0, 3, Eigen::Vector2d(0.0, 2.0)});
    q.push({5.0, 9, Eigen::Vector2d(0.0, 2.0)});

    auto[ok1, d1] = MafHlp::getDistanceToAgent(q, 3);
    ASSERT_TRUE(ok1);
    ASSERT_NEAR(d1.dist, 2.0, 0.0001);

    auto[ok2, d2] = MafHlp::getDistanceToAgent(q, 2);
    ASSERT_TRUE(ok2);
    ASSERT_NEAR(d2.dist, 1.0, 0.0001);

    auto[ok3, d3] = MafHlp::getDistanceToAgent(q, 9);
    ASSERT_TRUE(ok3);
    ASSERT_NEAR(d3.dist, 5.0, 0.0001);

    auto[ok4, d4] = MafHlp::getDistanceToAgent(q, 999);
    ASSERT_FALSE(ok4);
}

TEST(Helpers, AdjustVector)
{
    ASSERT_TRUE((MafHlp::adjustVectorScale(Eigen::Vector2d(12.0, 0.0), 5.0) -
                 Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    ASSERT_TRUE((MafHlp::adjustVectorScale(Eigen::Vector2d(2.0, 0.0), 5.0) -
                 Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    ASSERT_TRUE((MafHlp::adjustVectorScale(Eigen::Vector2d(0.0, 0.0), 5.0) -
                 Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    ASSERT_TRUE((MafHlp::adjustVectorScale(Eigen::Vector2d(-2.0, 0.0), 5.0) -
                 Eigen::Vector2d(-5.0, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Helpers, GetMax)
{
        std::vector<std::pair<double, Eigen::Vector2d>> vec = {{3.0, Eigen::Vector2d(1.0, 0.0)}, {6.0, Eigen::Vector2d(1.0, 1.0)},
                                                              {2.0, Eigen::Vector2d(4.0, 0.0)}};

        auto[maxVal, maxVec] = MafHlp::getMax(vec);

        ASSERT_NEAR(maxVal, 6.0, 0.0001);
        ASSERT_TRUE((Eigen::Vector2d(1.0, 1.0) - maxVec).isMuchSmallerThan(0.0001));
}


TEST(Helpers, TriangleOrientaion)
{
    Eigen::Vector2d p1(0.0, 0.0);
    Eigen::Vector2d p2(0.0, 1.0);
    Eigen::Vector2d p3(1.0, 0.0);
    Eigen::Vector2d pc3(100.0, 0.0);

    ASSERT_EQ(MafHlp::orientationOfTriangle(p1, p2, p3), MafHlp::TriOrientation::Clockwise);
    ASSERT_EQ(MafHlp::orientationOfTriangle(p3, p1, p2), MafHlp::TriOrientation::Clockwise);
    ASSERT_EQ(MafHlp::orientationOfTriangle(p2, p3, p1), MafHlp::TriOrientation::Clockwise);

    ASSERT_EQ(MafHlp::orientationOfTriangle(p1, p3, p2), MafHlp::TriOrientation::CounterClockwise);
    ASSERT_EQ(MafHlp::orientationOfTriangle(p2, p1, p3), MafHlp::TriOrientation::CounterClockwise);
    ASSERT_EQ(MafHlp::orientationOfTriangle(p3, p2, p1), MafHlp::TriOrientation::CounterClockwise);

    ASSERT_EQ(MafHlp::orientationOfTriangle(p1, p3, p3), MafHlp::TriOrientation::Collinear);
    ASSERT_EQ(MafHlp::orientationOfTriangle(p1, p3, pc3), MafHlp::TriOrientation::Collinear);
}
