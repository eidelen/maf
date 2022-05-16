
#include <gtest/gtest.h>
#include "message.h"

TEST(Message, ConstructDefault)
{
    auto m = new Message(0, 1, Message::Enable);
    ASSERT_EQ(m->senderId(), 0);
    ASSERT_EQ(m->receiverId(), 1);
    ASSERT_EQ(m->subject(), Message::Enable);
    delete m;
}

TEST(Message, Constructor)
{
    auto m = new Message(0, 1, Message::Enable, "ABC", {1.0, 2.0}, {4, 5});

    ASSERT_TRUE(m->textParam()=="ABC");

    ASSERT_EQ(m->doubleVecParam().size(), 2);
    ASSERT_NEAR(m->doubleVecParam()[0], 1.0, 0.000001);
    ASSERT_NEAR(m->doubleVecParam()[1], 2.0, 0.000001);

    ASSERT_EQ(m->intVecParam().size(), 2);
    ASSERT_EQ(m->intVecParam()[0], 4);
    ASSERT_EQ(m->intVecParam()[1], 5);

    delete m;
}

TEST(Message, ToString)
{
    auto m = new Message(99, 88, Message::Enable, "ABC", {1.0, 2.0}, {4, 5});

    std::string mStr = m->toString();

    ASSERT_TRUE(mStr.find("Enable") != std::string::npos);
    ASSERT_TRUE(mStr.find("99") != std::string::npos);
    ASSERT_TRUE(mStr.find("88") != std::string::npos);

    delete m;
}
