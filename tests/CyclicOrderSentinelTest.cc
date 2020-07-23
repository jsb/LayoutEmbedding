#include <gtest/gtest.h>

#include <LayoutEmbedding/CyclicOrderSentinel.hh>

TEST(CyclicOrderSentinelTest, TripletPositive1)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(1);
    cos.encounter(2);
    cos.encounter(3);
    EXPECT_TRUE(cos.valid());
}

TEST(CyclicOrderSentinelTest, TripletPositive2)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(3);
    cos.encounter(1);
    cos.encounter(2);
    EXPECT_TRUE(cos.valid());
}

TEST(CyclicOrderSentinelTest, TripletPositive3)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(2);
    cos.encounter(3);
    cos.encounter(1);
    EXPECT_TRUE(cos.valid());
}

TEST(CyclicOrderSentinelTest, TripletNegative1)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(1);
    cos.encounter(3);
    cos.encounter(2);
    EXPECT_FALSE(cos.valid());
}

TEST(CyclicOrderSentinelTest, TripletNegative2)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(2);
    cos.encounter(1);
    cos.encounter(3);
    EXPECT_FALSE(cos.valid());
}

TEST(CyclicOrderSentinelTest, TripletNegative3)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(3);
    cos.encounter(2);
    cos.encounter(1);
    EXPECT_FALSE(cos.valid());
}

TEST(CyclicOrderSentinelTest, Pair1)
{
    CyclicOrderSentinel<int> cos({1, 2});
    cos.encounter(1);
    cos.encounter(2);
    EXPECT_TRUE(cos.valid());
}

TEST(CyclicOrderSentinelTest, Pair2)
{
    CyclicOrderSentinel<int> cos({1, 2});
    cos.encounter(2);
    cos.encounter(1);
    EXPECT_TRUE(cos.valid());
}

TEST(CyclicOrderSentinelTest, IgnoreUnknownEntries)
{
    CyclicOrderSentinel<int> cos({1, 2, 3});
    cos.encounter(4); // should be ignored
    cos.encounter(2);
    cos.encounter(5); // should be ignored
    cos.encounter(3);
    cos.encounter(6); // should be ignored
    cos.encounter(1);
    cos.encounter(7); // should be ignored
    EXPECT_TRUE(cos.valid());
}
