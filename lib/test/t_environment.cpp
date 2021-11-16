
#include <gtest/gtest.h>
#include "environment.h"

TEST(Environment, Id)
{
    auto e = Environment::createEnvironment(9);
    ASSERT_EQ(e->id(), 9);
}

TEST(Environment, MovePossibleBaseImpl)
{
    auto e = Environment::createEnvironment(9);
    auto[possible, newPos] = e->possibleMove(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(5.0, 9.0));
    ASSERT_TRUE(possible);
    ASSERT_TRUE((newPos - Eigen::Vector2d(5.0, 9.0)).isMuchSmallerThan(0.0001));
}

TEST(Environment, AgentDistance)
{
    auto a = Agent::createAgent(4);
    auto b = Agent::createAgent(5);

    a->setPosition(Eigen::Vector2d(-4.0, 0.0));
    b->setPosition(Eigen::Vector2d(0.0, 3.0));

    Eigen::Vector2d res = Environment::computeDistance(a, b);

    ASSERT_TRUE((res - Eigen::Vector2d(4.0,3.0)).isMuchSmallerThan(0.0001));
}

TEST(Environment, DistanceMap)
{
    auto e = Environment::createEnvironment(9);

    // No entries in yet
    ASSERT_EQ(e->getAgentDistances().first.size(), 0);
    e->computeDistances();
    ASSERT_EQ(e->getAgentDistances().first.size(), 0);

    auto a = Agent::createAgent(2);
    a->setPosition(Eigen::Vector2d(2.0, 0.0));
    e->addAgent(a);

    auto b = Agent::createAgent(5);
    b->setPosition(Eigen::Vector2d(5.0, 0.0));
    e->addAgent(b);

    auto c = Agent::createAgent(20);
    c->setPosition(Eigen::Vector2d(20.0, 0.0));
    e->addAgent(c);

    e->computeDistances();
    auto[id, dists] = e->getAgentDistances();

    ASSERT_EQ(id.size(), 3);
    ASSERT_EQ(id.at(0), 2);
    ASSERT_EQ(id.at(1), 5);
    ASSERT_EQ(id.at(2), 20);

    ASSERT_NEAR(dists(0,0), 0.0, 0.00001);
    ASSERT_NEAR(dists(1,1), 0.0, 0.00001);
    ASSERT_NEAR(dists(2,2), 0.0, 0.00001);

    ASSERT_NEAR(dists(0,1), 3.0, 0.00001);
    ASSERT_NEAR(dists(0,2), 18.0, 0.00001);
    ASSERT_NEAR(dists(1,0), 3.0, 0.00001);
    ASSERT_NEAR(dists(2,0), 18.0, 0.00001);

    ASSERT_NEAR(dists(1,2), 15.0, 0.00001);
    ASSERT_NEAR(dists(2,1), 15.0, 0.00001);
}

#include <queue>

struct Di
{
    double d;
    std::string a;
};

class CmpDi
{
public:
    bool operator() (Di a, Di b)
    {
        return a.d > b.d;
    }
};

TEST(Environment, TestPriorityQueuesDistances)
{
    std::priority_queue<Di, std::vector<Di>, CmpDi> q;
    q.push({4.5, "abc"});
    q.push({1.2, "rst"});
    q.push({7.9, "opp"});

    while(!q.empty())
    {
        Di d = q.top();
        //std::cout << d.d << "   " << d.a << std::endl;
        q.pop();
    }
}

void compareDist(Environment::Distance a, Environment::Distance b)
{
    auto[d0, id0, vec0] = a;
    auto[d1, id1, vec1] = b;
    ASSERT_NEAR(d0, d1, 0.0001);
    ASSERT_TRUE((vec0 - vec1).isMuchSmallerThan(0.0001));
    ASSERT_EQ(id0, id1);
}

TEST(Environment, TestActualDistanceQueue)
{
    Environment::DistanceQueue q;
    q.push({4.2, 9, Eigen::Vector2d(4.2, 9.0)});
    q.push({7.5, 5, Eigen::Vector2d(7.5, 5.0)});
    q.push({-2.8, 12, Eigen::Vector2d(-2.8, 12.0)});

    compareDist(q.top(), {-2.8, 12, Eigen::Vector2d(-2.8, 12.0)});
    q.pop();
    compareDist(q.top(), {4.2, 9, Eigen::Vector2d(4.2, 9.0)});
    q.pop();
    compareDist(q.top(), {7.5, 5, Eigen::Vector2d(7.5, 5.0)});
    q.pop();
}


