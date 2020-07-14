// Copyright 2020 Intermodalics BVBA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OROCOS__RTT_ROS2_TOPICS__WAITABLE_HPP_
#define OROCOS__RTT_ROS2_TOPICS__WAITABLE_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <utility>

#include "rcl/guard_condition.h"
#include "rclcpp/context.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/waitable.hpp"

namespace rtt_ros2_topics
{

class Waitable
  : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Waitable)

  Waitable(
    std::function<bool()> func,
    rclcpp::Context::SharedPtr context)
  : func_(std::move(func)),
    context_(std::move(context)),
    rcl_guard_condition_(rcl_get_zero_initialized_guard_condition())
  {
    const rcl_ret_t ret = rcl_guard_condition_init(
      &rcl_guard_condition_,
      context_->get_rcl_context().get(),
      rcl_guard_condition_get_default_options());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  virtual ~Waitable() noexcept
  {
    rcl_ret_t ret = rcl_guard_condition_fini(&rcl_guard_condition_);
    if (RCL_RET_OK != ret) {
      try {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rtt_ros2_topics"),
          "Error in destruction of rcl guard condition: %s", exception.what());
      }
    }
  }

  void trigger()
  {
    has_work_ = true;
    // TODO(meyerj) Check whether the following call is safe in a real-time context
    // (e.g. Xenomai). For FastRTPS this locks an std::mutex and notifies a
    // std::condition_variable.
    const rcl_ret_t ret = rcl_trigger_guard_condition(&rcl_guard_condition_);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
  }

  /// Notify this Waitable instance about its pending destruction.
  /**
   * func_() must not be called anymore after cancel() returned.
   */
  void cancel() noexcept
  {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    func_ = decltype(func_) {};
  }

  size_t get_number_of_ready_guard_conditions() override
  {
    return 1u;
  }

  bool add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

    const rcl_ret_t ret = rcl_wait_set_add_guard_condition(
      wait_set, &rcl_guard_condition_, nullptr);
    return RCL_RET_OK == ret;
  }

  bool is_ready(rcl_wait_set_t *) override
  {
    return has_work_;
  }

  void execute() override
  {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    if (!func_) {return;}

    has_work_ = false;
    if (!func_()) {
      // eventually there is more work for us...
      trigger();
    }
  }

private:
  std::recursive_mutex reentrant_mutex_;
  std::mutex execute_mutex_;

  std::function<bool(void)> func_;
  rclcpp::Context::SharedPtr context_;

  std::atomic<bool> has_work_;
  rcl_guard_condition_t rcl_guard_condition_;
};

}  // namespace rtt_ros2_topics

#endif  // OROCOS__RTT_ROS2_TOPICS__WAITABLE_HPP_
