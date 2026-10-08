^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-10-08)
------------------
* NavState::get_safe(), and fixed races in NavState and RTTFBuffer
* PluginSwitcher, to reconfigure plugins at runtime
* Robot geometry (radius, inscribed radius, height) centralized in system_node
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling: make_transform_listener() and get_package_share_path() wrap the APIs that differ between distros
* Conda packages with pixi (pixi-build-ros)
* Contributors: Francisco Martín Rico

0.4.2 (2026-07-26)
------------------
* Added get_no_group in NavState
* get_by_type and get_to_vector in NavState
* Add default group. Add debug info
* Fix bug in NavState when updating an existing key via pointer
* Refactor PerceptionHandler
  PerceptionHandler now represents a single sensor input, not a sensor group
* Perception types and ops to easynav_sensors
* Fix segfault in some cases and reduce extrapolation to the future
* GPLv3 -> Apache 2.0
* Add the option to get the fuse() efective timestamp
* Improve navstate print including the time
* Add exact_time parameter to fuse to use the last TF if false
* Adjust process time to input times
* Fix TF stucks
* Get last time of the perceptions in a view
* Update tests
* Set robot_frame as default for the perception pipeline
* Add a base_footprint frame in TFInfo
* Hotfix: Remove remaining C++20/23 features
* Remove std::expected from MethodBase::initialize interface
* Optimize copies in Point Perception when flushing
* Added non-zero timeout in fuse
* Added operation to get latest perception, with valid TF or not
* Buffer in pointperceptions
* TFInfo in RTTFBuffer
* Add world frame
* Create header fro TFInfo
* Add vision_msgs/msg/Detection3DArray perception
* Cleanup unused headers
* Remove queue_size param, set queues to 1
* Merge new changes into rolling
* Finished collision checker
* Option to non-lazy ops
* Optimize filter and collapse
* Optimized downsample and delete risky constructors
* Lazy update in PointPerceptions
* Remove unused headers
* Optimization of fuse and collapse
* Add queue_size parameter
* First functional version
* Add namespace to yaets tracing macro
* GNSS Support
* Add missing dependencies
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.1.4 (2025-10-16)
------------------
* Fix compilation errors in jazzy
* Merge kilted version bump into rolling
* Merge kilted into rolling. Update version to 0.1.3
* Contributors: Francisco Miguel Moreno

0.1.3 (2025-10-12)
------------------
* New Constructor for PointPerceptionsView with only one perception
* Fix tf2_ros deprecation warnings
* Fix multiple perceptions
* Update defaul group var name
* Fix support for different perceptions types in blackboard
* Verbose fail in set. Set wit std::shared_ptr
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Remove some yaets traces
* Add operation to include new points in the PointPerceptionView pipeline
* Trigger planner if new goals appear
* Minor improvements (refactor and RT option)
* [WIP] Blackboard for NavSate
* Simplify blackboard with two operations
* Remove atomics
* Remove get_ref
* Refactor Perceptions
* BlacKboard v3
* Redesign with NavState
* Blackboard for NavState
* Improving concurrency with atomic actions
* Yaets traces
* Blackboard working
* Remove pcl_conversions in target_link_libraries
* Improving concurrency with atomic actions
* Fix ament_target_dependencies_deprecation
* Install bins in correct place
* Fix ament_target_dependencies_deprecation
* Remove timeout
* Fix bug when disabling tracing. Rename tracing CMake option
* Updated Doxygen description headers
* easynav_interfaces and initial approach for a goal manager
* NavigationControl interface and GoalManager
* [WIP] System Execution
* RTTFBuffer. Dictionary maps.
* Improve PerceptionsOps to PerceptionsOpsView
* Add fused_perceptions method
* Add chained operations in perceptions
* Changes to load plugins
* Isolate Perceptions type from sensor utility functions
* Base interfaces for module algorithms
* Works on Map Manager Types
* Install runtime binary targets under lib
* Add NavState structure
* Move common base types to easynav_common
* Rename namespaces to easynav
* Refactoring to nodes
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
