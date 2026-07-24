// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
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

/// \file
/// \brief Declaration of the base class MethodBase used in plugin-based EasyNav method components.

#ifndef EASYNAV_CORE__METHODBASE_HPP_
#define EASYNAV_CORE__METHODBASE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class MethodBase
 * @brief Base class for Easy Navigation method plugins.
 *
 * Provides lifecycle integration and update-rate management
 * for components such as localizers, controllers, or map managers.
 */
class MethodBase
{
public:
  /// @brief Default constructor.
  MethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~MethodBase() = default;

  /**
   * @brief Initializes the method with the given node and plugin name.
   *
   * Also reads update frequencies and sets up internal state.
   *
   * @param parent_node Shared pointer to the parent lifecycle node.
   * @param plugin_name Name of the plugin (used for parameters and logging).
   * @throws std::runtime_error on initialization failure.
   */
  virtual void initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name);

  /**
   * @brief Hook for custom setup logic in derived classes.
   *
   * Called from initialize(). Can be overridden to implement extra initialization steps.
   * @throws std::runtime_error on initialization failure.
   */
  virtual void on_initialize() {}

  /**
   * @brief Get a shared pointer to the parent lifecycle node.
   *
   * @return Shared pointer to the lifecycle node.
   */
  [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
  get_node() const;

  /**
   * @brief Get the name assigned to the plugin.
   *
   * @return Plugin name as a constant reference.
   */
  [[nodiscard]] const std::string &
  get_plugin_name() const;

  /**
   * @brief Check whether it is time to run a real-time update.
   *
   * Uses the configured real-time frequency to determine whether sufficient time has elapsed.
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2RunRT();

  /**
   * @brief Check whether it is time to run a normal (non-RT) update.
   *
   * Uses the configured frequency to determine whether sufficient time has elapsed.
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2Run();

  /**
   * @brief Mark that the real-time update has just run.
   *
   * Records the current time as the last RT execution timestamp. Call this right after
   * completing a successful RT iteration.
   *
   * @post @ref get_last_rt_execution_ts() reflects the time of this call.
   */
  void setRunRT();

  /**
   * @brief Mark that the normal (non-RT) update has just run.
   *
   * Records the current time as the last non-RT execution timestamp. Call this right after
   * completing a successful non-RT iteration.
   *
   * @post @ref get_last_execution_ts() reflects the time of this call.
   */
  void setRun();

  /**
   * @brief Get the timestamp of the last real-time execution.
   * @return Reference to the last RT execution time as recorded by @ref setRunRT().
   * @note If no RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_rt_execution_ts() const {return rt_last_ts_;}

  /**
   * @brief Get the timestamp of the last non-RT execution.
   * @return Reference to the last non-RT execution time as recorded by @ref setRun().
   * @note If no non-RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_execution_ts() const {return last_ts_;}

private:
  /// @brief Shared pointer to the parent lifecycle node.
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_;

  /// @brief Name assigned to the plugin.
  std::string plugin_name_;

  /// @brief Desired real-time and non-RT loop frequencies in Hz.
  float rt_frequency_, frequency_;

  /// @brief Timestamps of the last executions.
  rclcpp::Time rt_last_ts_, last_ts_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__METHODBASE_HPP_
