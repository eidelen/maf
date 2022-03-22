
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

void compareDist(Environment::Distance a, Environment::Distance b)
{
    auto[d0, id0, vec0] = a;
    auto[d1, id1, vec1] = b;
    ASSERT_NEAR(d0, d1, 0.0001);
    ASSERT_TRUE((vec0 - vec1).isMuchSmallerThan(0.0001));
    ASSERT_EQ(id0, id1);
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

TEST(Environment, TestDistanceMap)
{
    Environment::DistanceMap m;

    // add content to queues
    m[5].push({7.5, 55, Eigen::Vector2d(7.5, 55.0)});
    m[5].push({3.5, 555, Eigen::Vector2d(3.5, 555.0)});
    m[8].push({7.5, 88, Eigen::Vector2d(7.5, 88.0)});
    m[8].push({3.5, 888, Eigen::Vector2d(3.5, 888.0)});

    compareDist(m[5].top(), {3.5, 555, Eigen::Vector2d(3.5, 555.0)});
    compareDist(m[8].top(), {3.5, 888, Eigen::Vector2d(3.5, 888.0)});

    // check when using reference
    Environment::DistanceQueue& m5Ref = m[5];
    compareDist(m5Ref.top(), {3.5, 555, Eigen::Vector2d(3.5, 555.0)});

    // pop from reference
    m5Ref.pop();

    compareDist(m[5].top(), {7.5, 55, Eigen::Vector2d(7.5, 55.0)});
    compareDist(m5Ref.top(), {7.5, 55, Eigen::Vector2d(7.5, 55.0)});
}

TEST(Environment, DistanceMap)
{
    auto e = Environment::createEnvironment(9);

    // No entries in yet
    ASSERT_EQ(e->getAgentDistances().size(), 0);
    e->computeDistances();
    ASSERT_EQ(e->getAgentDistances().size(), 0);

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
    Environment::DistanceMap& dMap = e->getAgentDistances();

    ASSERT_EQ(dMap.size(), 3);

    // Check distances of Agent id = 2
    Environment::DistanceQueue& qA2 = dMap[2];
    ASSERT_EQ(qA2.size(), 2);
    compareDist(qA2.top(), {3.0, 5, Eigen::Vector2d(3.0, 0.0)});
    qA2.pop();
    compareDist(qA2.top(), {18.0, 20, Eigen::Vector2d(18.0, 0.0)});

    // Check distances of Agent id = 5
    Environment::DistanceQueue& qA5 = dMap[5];
    ASSERT_EQ(qA5.size(), 2);
    compareDist(qA5.top(), {3.0, 2, Eigen::Vector2d(-3.0, 0.0)});
    qA5.pop();
    compareDist(qA5.top(), {15.0, 20, Eigen::Vector2d(15.0, 0.0)});

    // Check distances of Agent id = 20
    Environment::DistanceQueue& qA20 = dMap[20];
    ASSERT_EQ(qA20.size(), 2);
    compareDist(qA20.top(), {15.0, 5, Eigen::Vector2d(-15.0, 0.0)});
    qA20.pop();
    compareDist(qA20.top(), {18.0, 2, Eigen::Vector2d(-18.0, 0.0)});
}

TEST(Environment, DistanceMapSubAgents)
{
    auto e = Environment::createEnvironment(9);

    // No entries in yet
    ASSERT_EQ(e->getAgentDistances().size(), 0);
    e->computeDistances();
    ASSERT_EQ(e->getAgentDistances().size(), 0);

    auto a = Agent::createAgent(2);
    a->setPosition(Eigen::Vector2d(2.0, 0.0));
    e->addAgent(a);

    // c is subagent of b!
    auto b = Agent::createAgent(5);
    b->setPosition(Eigen::Vector2d(5.0, 0.0));
    auto c = Agent::createAgent(20);
    c->setPosition(Eigen::Vector2d(20.0, 0.0));
    b->addSubAgent(c);
    e->addAgent(b);

    e->computeDistances();
    Environment::DistanceMap& dMap = e->getAgentDistances();

    ASSERT_EQ(dMap.size(), 3);

    // Check distances of Agent id = 2
    Environment::DistanceQueue& qA2 = dMap[2];
    ASSERT_EQ(qA2.size(), 2);
    compareDist(qA2.top(), {3.0, 5, Eigen::Vector2d(3.0, 0.0)});
    qA2.pop();
    compareDist(qA2.top(), {18.0, 20, Eigen::Vector2d(18.0, 0.0)});

    // Check distances of Agent id = 5
    Environment::DistanceQueue& qA5 = dMap[5];
    ASSERT_EQ(qA5.size(), 2);
    compareDist(qA5.top(), {3.0, 2, Eigen::Vector2d(-3.0, 0.0)});
    qA5.pop();
    compareDist(qA5.top(), {15.0, 20, Eigen::Vector2d(15.0, 0.0)});

    // Check distances of Agent id = 20
    Environment::DistanceQueue& qA20 = dMap[20];
    ASSERT_EQ(qA20.size(), 2);
    compareDist(qA20.top(), {15.0, 5, Eigen::Vector2d(-15.0, 0.0)});
    qA20.pop();
    compareDist(qA20.top(), {18.0, 2, Eigen::Vector2d(-18.0, 0.0)});
}

TEST(Environment, MessagesContainer)
{
    auto e = Environment::createEnvironment(9);

    // no messages yet
    ASSERT_EQ(e->getMessages(1).size(), 0);

    // 2 messages for 1
    std::shared_ptr<Message> a(new Message(0, 1, Message::Disable));
    std::shared_ptr<Message> b(new Message(0, 1, Message::Enable));
    e->sendMessage(a);
    e->sendMessage(b);
    ASSERT_EQ(e->getMessages(1).size(), 2);

    // reading messages removes them
    auto ra = e->getMessages(1).front();
    e->getMessages(1).pop();
    ASSERT_EQ(ra->senderId(), 0);
    ASSERT_EQ(ra->receiverId(), 1);
    ASSERT_EQ(ra->subject(), Message::Disable);
    auto rb = e->getMessages(1).front();
    e->getMessages(1).pop();
    ASSERT_EQ(rb->senderId(), 0);
    ASSERT_EQ(rb->receiverId(), 1);
    ASSERT_EQ(rb->subject(), Message::Enable);
    ASSERT_EQ(e->getMessages(1).size(), 0);
}


TEST(Environment, SendAndReceiveMessages)
{
    auto e = Environment::createEnvironment(9);
    auto a1 = Agent::createAgent(10);
    auto a2 = Agent::createAgent(20);
    a1->setEnvironment(e);
    a2->setEnvironment(e);
    e->addAgent(a1);
    e->addAgent(a2);

    // both enabled at start
    e->update(10);
    ASSERT_TRUE(a1->getEnabled());
    ASSERT_TRUE(a2->getEnabled());

    // send disable to wrong address
    a1->sendMessage(99, Message::Disable);
    e->update(10);
    ASSERT_TRUE(a1->getEnabled());
    ASSERT_TRUE(a2->getEnabled());

    // send disable to agent a2
    a1->sendMessage(20, Message::Disable);
    e->update(10);
    ASSERT_TRUE(a1->getEnabled());
    ASSERT_FALSE(a2->getEnabled());

    // send disable to agent a1 too
    a2->sendMessage(10, Message::Disable);
    e->update(10);
    ASSERT_FALSE(a1->getEnabled());
    ASSERT_FALSE(a2->getEnabled());

    // enable both again
    a2->sendMessage(10, Message::Enable);
    a1->sendMessage(20, Message::Enable);
    e->update(10);
    ASSERT_TRUE(a1->getEnabled());
    ASSERT_TRUE(a2->getEnabled());
}

TEST(Environment, DistanceMapDisabledAgents)
{
    auto e = Environment::createEnvironment(9);

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
    Environment::DistanceMap& dMap = e->getAgentDistances();

    ASSERT_EQ(dMap.size(), 3);

    // disable agent c
    c->setEnabled(false);

    e->computeDistances();
    Environment::DistanceMap& dMapReduced = e->getAgentDistances();

    ASSERT_EQ(dMapReduced.size(), 2);

    // not available
    Environment::DistanceQueue& qA3 = dMapReduced[3];
    ASSERT_EQ(qA3.size(), 0);
}

class CircEnv: public Environment
{
public:
    CircEnv(unsigned int id): Environment(id) {}
    virtual ~CircEnv() {}
    virtual std::pair<bool, Eigen::Vector2d> possibleMove(const Eigen::Vector2d& origin, const Eigen::Vector2d& destination) const override
    {
        // Circular environmet with radius 10. If move not possible, return
        // previous position.
        double radius = 10.0;
        if(destination.norm() < radius)
            return {true, destination};
        else
            return {false, origin};
    }

};

TEST(Environment, DistanceToEnvBorder)
{
    auto e = std::shared_ptr<CircEnv>(new CircEnv(4));

    // 5 m to border in x-dir
    double d1 = e->distanceToEnvironmentBorder(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(1.0, 0.0), 0.1, 30.0);
    ASSERT_NEAR(d1, 5.0, 0.0001);

    // 15 m in y-dir
    double d2 = e->distanceToEnvironmentBorder(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(-1.0, 0.0), 0.1, 30.0);
    ASSERT_NEAR(d2, 15.0, 0.0001);

    // out of reach
    double d3 = e->distanceToEnvironmentBorder(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(1.0, 0.0), 0.1, 2.0);
    ASSERT_NEAR(d3, 2.0, 0.0001);
    double d4 = e->distanceToEnvironmentBorder(Eigen::Vector2d(5.0, 0.0), Eigen::Vector2d(-1.0, 0.0), 0.1, 2.0);
    ASSERT_NEAR(d4, 2.0, 0.0001);

    // starting at invalid position
    double d5 = e->distanceToEnvironmentBorder(Eigen::Vector2d(15.0, 0.0), Eigen::Vector2d(-1.0, 0.0), 0.1, 2.0);
    ASSERT_NEAR(d5, 0.0, 0.0001);
}

