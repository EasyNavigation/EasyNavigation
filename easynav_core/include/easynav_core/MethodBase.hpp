// Copyright 2025 Intelligent Robotics Lab
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

/// \file
/// \brief Declaration of the base class MethodBase used in plugin-based EasyNav method components.

#ifndef EASYNAV_CORE__METHODBASE_HPP_
#define EASYNAV_CORE__METHODBASE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/types/RobotGeometry.hpp"
#include "easynav_core/RateMonitor.hpp"

namespace easynav
{

/**
 * @class MethodBase
 * @brief Base class for Easy Navigation method plugins.
 *
 * Provides lifecycle integration and update-rate management
 * for components such as localizers, controllers, or map managers.
 *
 * Each component runs its RT update at "<plugin>.rt_freq" and its non-RT update at
 * "<plugin>.freq" (Hz). The system cycles ("system_node.rt_freq" and "system_node.freq") only
 * decide how often it is checked whether it is time to run: they must be at least the
 * components' frequencies. The schedule does not drift: a component at 30 Hz checked at 50 Hz
 * runs 30 times per second. Whether each component keeps its frequency is reported as the
 * diagnostics "<plugin>.rt_rate" and "<plugin>.rate" (see RateMonitor): a WARN when not.
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
   * @brief The robot's geometry ("system_node.robot_geometry.*").
   * @param legacy This plugin's deprecated parameters for each field, relative to its name
   * (e.g. "robot_radius" for "<plugin_name>.robot_radius"): applied, with a warning, unless
   * "robot_geometry" configures that field.
   */
  [[nodiscard]] RobotGeometry get_robot_geometry(const LegacyRobotGeometryNames & legacy = {});

  /**
   * @brief Check whether it is time to run a real-time update.
   *
   * True once per period of "<plugin>.rt_freq", on a fixed schedule: a check that comes late
   * does not delay the next run. More than a period behind, the schedule restarts from now.
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2RunRT();

  /**
   * @brief Check whether it is time to run a normal (non-RT) update.
   *
   * Same as @ref isTime2RunRT(), with "<plugin>.freq".
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2Run();

  /**
   * @brief Mark that the real-time update has just run.
   *
   * Records the run, for its timestamp and its rate. Call it on every run. A run that
   * @ref isTime2RunRT() did not schedule (e.g. triggered) restarts the schedule from now.
   *
   * @post @ref get_last_rt_execution_ts() reflects the time of this call.
   */
  void setRunRT();

  /**
   * @brief Mark that the normal (non-RT) update has just run.
   *
   * Same as @ref setRunRT(), for the non-RT update.
   *
   * @post @ref get_last_execution_ts() reflects the time of this call.
   */
  void setRun();

  /**
   * @brief Report whether the RT update keeps its frequency ("diagnostics.<plugin>.rt_rate").
   *
   * Call it on every RT cycle, before deciding whether to run. It writes the first status and
   * then only its changes. Not keeping the frequency is only a WARN (reported, not mitigated);
   * after RateMonitor::kMaxSlowWindows slow windows in a row, its message says for how long.
   */
  void report_rt_rate(NavState & nav_state);

  /// @brief Same as @ref report_rt_rate(), for the non-RT update ("diagnostics.<plugin>.rate").
  void report_rate(NavState & nav_state);

  /**
   * @brief Forget the measured rates, e.g. on activation: the time inactive is not slowness.
   *
   * The nodes call it from on_activate(); the status reported next is OK.
   */
  void reset_rate_monitors();

  /// @brief Rate monitor of the RT update.
  [[nodiscard]] const RateMonitor & get_rt_rate_monitor() const {return rt_cycle_.monitor;}

  /// @brief Rate monitor of the non-RT update.
  [[nodiscard]] const RateMonitor & get_rate_monitor() const {return cycle_.monitor;}

  /**
   * @brief Get the timestamp of the last real-time execution.
   * @return Reference to the last RT execution time as recorded by @ref setRunRT().
   * @note If no RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_rt_execution_ts() const {return rt_cycle_.last_ts;}

  /**
   * @brief Get the timestamp of the last non-RT execution.
   * @return Reference to the last non-RT execution time as recorded by @ref setRun().
   * @note If no non-RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_execution_ts() const {return cycle_.last_ts;}

private:
  /// @brief Schedule and rate of one update (RT or non-RT).
  struct Cycle
  {
    std::string parameter;  // "rt_freq" or "freq"
    double frequency {10.0};
    rclcpp::Time last_ts, next_ts;
    bool scheduled {false};  // isTime2Run*() said so, and no run yet
    RateMonitor monitor;
    std::string key, name;  // Diagnostic
    std::optional<RateMonitor::Status> reported;
  };

  bool is_time_to_run(Cycle & cycle);
  void set_run(Cycle & cycle);
  void report(Cycle & cycle, NavState & nav_state);

  /// @brief Shared pointer to the parent lifecycle node.
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_;

  /// @brief Name assigned to the plugin.
  std::string plugin_name_;

  Cycle rt_cycle_, cycle_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__METHODBASE_HPP_
