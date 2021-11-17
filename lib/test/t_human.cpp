
#include <gtest/gtest.h>
#include "human.h"
#include "environment.h"

TEST(Human, MaxSpeed)
{
    auto env = Environment::createEnvironment(3);
    auto h = Human::createHuman(11, 10.0, 1.0);
    h->disableReacting(true);
    h->setEnvironment(env);

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

    // check velocity does not go over limit
    h->setVelocity(Eigen::Vector2d(0.0, 0.0));
    h->setAcceleration(Eigen::Vector2d(1.0, 0.0));

    h->move(5.0);

    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(5.0, 0.0)).isMuchSmallerThan(0.0001));

    h->move(5.0);

    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));

    h->move(0.1);

    // max speed reached -> limit to 10
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));
}

TEST(Human, SlowDown)
{
    auto env = Environment::createEnvironment(3);
    auto h = Human::createHuman(11, 10.0, 4.0);
    h->setEnvironment(env);

    h->setVelocity(Eigen::Vector2d(10.0, 0.0));
    ASSERT_TRUE((h->getVelocity() - Eigen::Vector2d(10.0, 0.0)).isMuchSmallerThan(0.0001));

    // human gets slower, as no other agent around
    double lastSpeed = h->getVelocity().norm();

    h->move(1.0);
    ASSERT_GT(lastSpeed, h->getVelocity().norm());
    lastSpeed = h->getVelocity().norm();
    std::cout << lastSpeed << std::endl;

    h->move(1.0);
    ASSERT_GT(lastSpeed, h->getVelocity().norm());
    lastSpeed = h->getVelocity().norm();
    std::cout << lastSpeed << std::endl;

    h->move(1.0);
    std::cout << lastSpeed << std::endl;
    ASSERT_GT(lastSpeed, h->getVelocity().norm());

}
