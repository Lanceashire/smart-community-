#include "sentry_motion_manager/command_buffer.hpp"
#include "gtest/gtest.h"

using sentry_motion_manager::CommandBuffer;
using sentry_motion_manager::MotionCommand;
rclcpp::Time timeAt(double t) { return rclcpp::Time(static_cast<int64_t>(t * 1e9)); }
MotionCommand commandAt(double t, double v)
{
  MotionCommand c;
  c.valid = true;
  c.twist.header.stamp = timeAt(t);
  c.twist.twist.linear.x = v;
  return c;
}
TEST(CommandBuffer, ClockSkewDoesNotDiscardFreshCommand)
{
  CommandBuffer b;
  b.push(commandAt(51.0, .095), timeAt(51.0), .25);
  b.push(commandAt(51.1, 0.), timeAt(51.099), .25);
  ASSERT_NE(b.at(timeAt(51.099), .25), nullptr);
  EXPECT_DOUBLE_EQ(b.at(timeAt(51.099), .25)->twist.twist.linear.x, .095);
  EXPECT_DOUBLE_EQ(b.at(timeAt(51.1), .25)->twist.twist.linear.x, 0.);
}
TEST(CommandBuffer, FutureMotionNeverExecutedEarly)
{
  CommandBuffer b;
  b.push(commandAt(10.02, .26), timeAt(10.), .25);
  EXPECT_EQ(b.at(timeAt(10.019), .25), nullptr);
  ASSERT_NE(b.at(timeAt(10.02), .25), nullptr);
}
TEST(CommandBuffer, PendingDoesNotExtendPreviousSourceLifetime)
{
  CommandBuffer b;
  b.push(commandAt(10., .26), timeAt(10.), .25);
  b.push(commandAt(10.28, .1), timeAt(10.24), .25);
  EXPECT_NE(b.at(timeAt(10.25), .25), nullptr);
  EXPECT_EQ(b.at(timeAt(10.251), .25), nullptr);
  EXPECT_NE(b.at(timeAt(10.28), .25), nullptr);
  EXPECT_EQ(b.at(timeAt(10.531), .25), nullptr);
}
TEST(CommandBuffer, ReplayCannotReplaceNewerStop)
{
  CommandBuffer b;
  b.push(commandAt(10.1, 0.), timeAt(10.1), .25);
  b.push(commandAt(10., .26), timeAt(10.11), .25);
  ASSERT_NE(b.at(timeAt(10.11), .25), nullptr);
  EXPECT_DOUBLE_EQ(b.at(timeAt(10.11), .25)->twist.twist.linear.x, 0.);
}
TEST(CommandBuffer, FarFutureAndStaleCommandsDoNotPoisonBuffer)
{
  CommandBuffer b;
  b.push(commandAt(10., .1), timeAt(10.), .25);
  b.push(commandAt(100., .26), timeAt(10.01), .25);
  b.push(commandAt(9., .26), timeAt(10.01), .25);
  b.push(commandAt(10.02, 0.), timeAt(10.02), .25);
  ASSERT_NE(b.at(timeAt(10.02), .25), nullptr);
  EXPECT_DOUBLE_EQ(b.at(timeAt(10.02), .25)->twist.twist.linear.x, 0.);
}
TEST(CommandBuffer, NewFutureSamplesRetainLastEligibleSource)
{
  CommandBuffer b;
  b.push(commandAt(10., .1), timeAt(10.), .25);
  b.push(commandAt(10.03, .2), timeAt(10.001), .25);
  b.push(commandAt(10.04, .26), timeAt(10.002), .25);
  ASSERT_NE(b.at(timeAt(10.002), .25), nullptr);
  EXPECT_DOUBLE_EQ(b.at(timeAt(10.002), .25)->twist.twist.linear.x, .1);
}
