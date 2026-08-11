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

#include <string>
#include <atomic>
#include <thread>
#include <chrono>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/SystemNode.hpp"
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_common/YTSession.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::atomic_bool stop{false};

  std::thread rt_thread;
  {
    // Executors live in this scope and will be destroyed after join().
    rclcpp::executors::SingleThreadedExecutor exe_nort;
    rclcpp::executors::SingleThreadedExecutor exe_rt;

    auto system_node = easynav::SystemNode::make_shared();

    exe_nort.add_node(system_node->get_node_base_interface());
    exe_rt.add_callback_group(system_node->get_real_time_cbg(),
                              system_node->get_node_base_interface());

    auto tf_node = rclcpp::Node::make_shared("tf_node");

    auto tf_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    auto tf_buffer = easynav::RTTFBuffer::getInstance(tf_clock);

    for (auto & node : system_node->get_system_nodes()) {
      exe_nort.add_node(node.second.node_ptr->get_node_base_interface());
      if (node.second.realtime_cbg != nullptr) {
        exe_rt.add_callback_group(node.second.realtime_cbg,
                                  node.second.node_ptr->get_node_base_interface());
      }
    }

    // Lifecycle: configure -> activate
    system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (system_node->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(system_node->get_logger(), "Unable to configure EasyNav");
      rclcpp::shutdown();
      return 1;
    }
    system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (system_node->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(system_node->get_logger(), "Unable to activate EasyNav");
      rclcpp::shutdown();
      return 1;
    }

    bool use_real_time = true;
    system_node->declare_parameter("use_real_time", use_real_time);
    system_node->get_parameter("use_real_time", use_real_time);

    // Get rt and nonrt rates
    double rt_freq = 200.0;
    system_node->declare_parameter("rt_freq", rt_freq);
    system_node->get_parameter("rt_freq", rt_freq);
    double freq = 200.0;
    system_node->declare_parameter("freq", freq);
    system_node->get_parameter("freq", freq);

    // Get spin duration timeout for both threads
    double spin_time_rt = 0.001;
    system_node->declare_parameter("spin_time_rt", spin_time_rt);
    system_node->get_parameter("spin_time_rt", spin_time_rt);
    double spin_time_nort = 0.001;
    system_node->declare_parameter("spin_time_nort", spin_time_nort);
    system_node->get_parameter("spin_time_nort", spin_time_nort);

    // Convert spin timeouts from seconds to nanoseconds and cast to chrono type
    const auto spin_duration_rt = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(spin_time_rt)
    );

    const auto spin_duration_nort = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(spin_time_nort)
    );

    // Cooperative shutdown on SIGINT
    rclcpp::on_shutdown([&](){
        stop.store(true, std::memory_order_relaxed);
        exe_rt.cancel();
        exe_nort.cancel();
    });

    // RT thread
    rt_thread = std::thread(
      [&, tf_node, tf_buffer, system_node, use_real_time]() {
        if (use_real_time) {
          RCLCPP_INFO(system_node->get_logger(), "Selected Real-Time");
          sched_param sch; sch.sched_priority = 80;
          if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
            RCLCPP_WARN(system_node->get_logger(),
              "Failed to set Real Time. Running with normal priority.");
          }
        } else {
          RCLCPP_INFO(system_node->get_logger(), "Selected NO Real-Time");
        }

        tf2_ros::TransformListener tf_listener(*tf_buffer, *tf_node, true);

        rclcpp::WallRate rate(rt_freq);
        while (!stop.load(std::memory_order_relaxed)) {
          if (system_node->get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
          {
            system_node->system_cycle_rt();
          }
          {
            EASYNAV_TRACE_NAMED_EVENT("easynav_system::spin_rt");
            exe_rt.spin_all(spin_duration_rt);
          }
          rate.sleep();
        }
      });

    // Non-RT loop
    rclcpp::WallRate rate(freq);
    while (!stop.load(std::memory_order_relaxed)) {

      if (system_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        system_node->system_cycle();
      }
      {
        EASYNAV_TRACE_NAMED_EVENT("easynav_system::spin_nort");
        exe_nort.spin_all(spin_duration_nort);
      }
      rate.sleep();
    }

    // Ensure stop flag visible and cancel executors (idempotent)
    stop.store(true, std::memory_order_relaxed);
    exe_rt.cancel();
    exe_nort.cancel();
  }


  // Wait the RT thread to finish before shutting down ROS.
  if (rt_thread.joinable()) {
    rt_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
