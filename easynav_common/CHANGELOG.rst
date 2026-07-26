^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-07-26)
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
  API to get perception times
* Add the option to get the fuse() efective timestamp
* Improve navstate print including the time
* Add exact_time parameter to fuse to use the last TF if false
* Adjust process time to input times
* Fix TF stucks
* Get last ime of the perceptions in a view
* Set robot_frame as default for the perception pipeline
* Add a bas_footprint frame in TFInfo
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.3.2 (2025-12-18)
------------------
* Hotfix: Remove remaining C++20/23 features
* Contributors: Francisco Miguel Moreno

0.3.1 (2025-12-17)
------------------
* Downgrade from C++23 features
* Remove std::expected from MethodBase::initialize interface
* TF Refactor
* Unify TF configuration
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.3.0 (2025-12-01)
------------------
* Add vision_msgs/msg/Detection3DArray perception
* Cleanup unused headers
* Reshape execution and sensor handling
* Finished collision checker
* Optimize pointperceptionview
* Option to non-lazy ops
* Optimize filter and collapse
* Optimized downsample and delete risky constructors
* Lazy update in PointPerceptions
* Allow using yaets tracing macros from outside the easynav namespace
* Add namespace to yaets tracing macro
* Allow using yaets tracing macros from outside the easynav namespace
* GNSS Support
* Add missing dependencies
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.2.0 (2025-12-01)
------------------
* Merge rolling into jazzy

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
