// Copyright 2026 Intelligent Robotics Lab
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
/// \brief Declaration of the DummyRecoveryManager class.

#ifndef EASYNAV_RECOVERY__DUMMYRECOVERYMANAGER_HPP_
#define EASYNAV_RECOVERY__DUMMYRECOVERYMANAGER_HPP_

#include <atomic>
#include <cstdint>

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/RecoveryManagerBase.hpp"

namespace easynav
{

/**
 * @class DummyRecoveryManager
 * @brief A "dummy" recovery system: it never diagnoses, commands or asks anything.
 *
 * Loaded by RecoveryManagerNode when "recovery_manager.plugin" is not set, so EasyNav runs with
 * no recovery unless one is configured. It serves as an example of the minimum a
 * RecoveryManagerBase implementation needs, and counts its calls so tests can check what the
 * host forwards to it.
 */
class DummyRecoveryManager : public easynav::RecoveryManagerBase
{
public:
  DummyRecoveryManager() = default;
  ~DummyRecoveryManager() = default;

  /// @brief Called when EasyNav is activated.
  void on_activate() override {active_ = true;}

  /// @brief Called when EasyNav is deactivated.
  void on_deactivate() override {active_ = false;}

  /// @brief Whether EasyNav is active, as forwarded by the host.
  [[nodiscard]] bool is_active() const {return active_;}

  /// @brief Number of non-RT cycles run.
  [[nodiscard]] uint64_t get_update_count() const {return update_count_;}

  /// @brief Number of RT cycles run.
  [[nodiscard]] uint64_t get_update_rt_count() const {return update_rt_count_;}

protected:
  /// @brief Does nothing.
  void update(NavState & nav_state) override;

  /// @brief Does nothing: never commands the robot.
  bool update_rt(NavState & nav_state) override;

private:
  std::atomic<bool> active_ {false};
  std::atomic<uint64_t> update_count_ {0};
  std::atomic<uint64_t> update_rt_count_ {0};
};

}  // namespace easynav

#endif  // EASYNAV_RECOVERY__DUMMYRECOVERYMANAGER_HPP_
