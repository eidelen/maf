
#include <gtest/gtest.h>
#include "human.h"

TEST(Human, MaxSpeed)
{
    auto h = Human::createHuman(11, 10.0, 1.0);
/*
    // check set
    h->setVelocity(Eigen::Vector2d(0.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(0.0, 0.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.0, 0.0)).isMuchSmallerThan(0.0001));

    h->setVelocity(Eigen::Vector2d(0.5, 0.6));
    h->setAcceleration(Eigen::Vector2d(0.1, 0.2));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(0.5, 0.6)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.1, 0.2)).isMuchSmallerThan(0.0001));

    h->setVelocity(Eigen::Vector2d(11.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(0.0, 2.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
    ASSERT_TRUE((h->getAcceleration() - Eigen::Vector2d(0.0, 1.0)).isMuchSmallerThan(0.0001));
*/
    // check velocity does not go over limit
    h->setVelocity(Eigen::Vector2d(0.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(1.0, 0.0));

    h->move(5.0);

   // std::cout << h->getVelocity()  << std::endl;

   //ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));


}
