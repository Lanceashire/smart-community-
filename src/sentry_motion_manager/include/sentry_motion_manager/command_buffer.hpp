#ifndef SENTRY_MOTION_MANAGER__COMMAND_BUFFER_HPP_
#define SENTRY_MOTION_MANAGER__COMMAND_BUFFER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sentry_motion_manager/motion_types.hpp"

namespace sentry_motion_manager
{
// A source may receive the next /clock sample before this node. Retain the
// still-fresh command until the new timestamp becomes eligible; never execute
// a future command and never extend either command's source-time lifetime.
class CommandBuffer
{
public:
  void push(const MotionCommand & command, const rclcpp::Time & now, double timeout)
  {
    const auto stamp = rclcpp::Time(command.twist.header.stamp, now.get_clock_type());
    const double age = (now - stamp).seconds();
    if (!command.valid || age < -0.05 || age > timeout) {
      return;
    }
    if (latest_.valid) {
      const auto latest_stamp = rclcpp::Time(latest_.twist.header.stamp, now.get_clock_type());
      if (stamp < latest_stamp) {
        return;
      }
      if (fresh(latest_, now, timeout)) {
        previous_ = latest_;
      }
    }
    latest_ = command;
  }

  const MotionCommand * at(const rclcpp::Time & now, double timeout) const
  {
    if (fresh(latest_, now, timeout)) {
      return &latest_;
    }
    if (latest_.valid &&
      rclcpp::Time(latest_.twist.header.stamp, now.get_clock_type()) > now &&
      fresh(previous_, now, timeout))
    {
      return &previous_;
    }
    return nullptr;
  }

private:
  static bool fresh(const MotionCommand & command, const rclcpp::Time & now, double timeout)
  {
    const auto age = (now - rclcpp::Time(command.twist.header.stamp, now.get_clock_type())).seconds();
    return command.valid && age >= 0.0 && age <= timeout;
  }
  MotionCommand latest_{};
  MotionCommand previous_{};
};
}  // namespace sentry_motion_manager
#endif  // SENTRY_MOTION_MANAGER__COMMAND_BUFFER_HPP_
