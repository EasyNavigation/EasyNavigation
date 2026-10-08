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
/// \brief Declaration of the RateMonitor class.

#ifndef EASYNAV_CORE__RATEMONITOR_HPP_
#define EASYNAV_CORE__RATEMONITOR_HPP_

namespace easynav
{

/**
 * @class RateMonitor
 * @brief Checks that a component runs at its configured frequency.
 *
 * Runs are counted in windows of max(kMinWindow, kWindowPeriods periods). A window is slow if
 * it holds fewer than kMinRatio of the expected runs (one run of margin, for the window edges).
 * The status is WARN after a slow window, ERROR after kMaxSlowWindows slow windows in a row,
 * and OK again after a window on rate.
 *
 * Times are in seconds, of any clock that does not go back (a jump back restarts the window).
 * A gap without update() calls counts: if a blocked component held up its cycle for several
 * windows, that is that many slow windows. Call reset() when the gap is legitimate (e.g. on
 * activation, after being inactive).
 */
class RateMonitor
{
public:
  enum class Status {OK, WARN, ERROR};

  static constexpr double kMinRatio = 0.9;
  static constexpr int kMaxSlowWindows = 3;
  static constexpr double kMinWindow = 1.0;
  static constexpr int kWindowPeriods = 10;

  /// @brief Sets the expected frequency (Hz, > 0) and resets.
  void configure(double frequency);

  /// @brief Forgets the runs and the status.
  void reset();

  /// @brief Records a run at time now.
  void run(double now);

  /**
   * @brief Checks the rate at time now. Call it periodically (e.g. each scheduling check).
   * @return The status, updated when a window closes.
   */
  Status update(double now);

  [[nodiscard]] Status status() const {return status_;}
  [[nodiscard]] double frequency() const {return frequency_;}
  /// @brief Rate measured in the last closed window (Hz), or 0 if none closed yet.
  [[nodiscard]] double rate() const {return rate_;}
  [[nodiscard]] double window() const {return window_;}
  [[nodiscard]] int slow_windows() const {return slow_windows_;}

private:
  void start_window(double now);

  double frequency_ {1.0};
  double window_ {kMinWindow};
  bool started_ {false};
  double window_start_ {0.0};
  double last_update_ {0.0};
  int runs_ {0};
  double rate_ {0.0};
  int slow_windows_ {0};
  Status status_ {Status::OK};
};

}  // namespace easynav

#endif  // EASYNAV_CORE__RATEMONITOR_HPP_
