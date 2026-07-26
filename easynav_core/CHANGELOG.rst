^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-07-26)
------------------
* get_by_type and get_to_vector in NavState
* Refactor PerceptionHandler
  PerceptionHandler now represents a single sensor input, not a sensor group
* Fix test bug in plugin class loader
* Increase coverage
* Perception types and ops to easynav_sensors
* GPLv3 -> Apache 2.0
* Adjust process time to input times
* Sync time for markers with its the collision percetion time
* Set robot_frame as default for collision checker
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
* Add README.md with base classes description
* Set collision checker disabled by default
* Cleanup unused headers
* Reshape execution and sensor handling
* Add warning when exceeding the target cycle time
* Finished collision checker
* Reset method execution time when triggered
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.2.0 (2025-12-01)
------------------
* Add README.md with base classes description
* Remove unused include and random
* Set collision checker disabled by default
* Cleanup unused headers

0.1.4 (2025-10-16)
------------------
* Merge kilted version bump into rolling
* Merge kilted into rolling. Update version to 0.1.3
* Contributors: Francisco Miguel Moreno

0.1.3 (2025-10-12)
------------------

0.1.2 (2025-09-26)
------------------
* Trigger planner if new goals appear
* Multi-robot support by namespacing TFs
* Use tf_prefix instead of tf_namespace
* [WIP] Blackboard for NavSate
* Redesing with NavState
* Yaets traces
* Fix CMake targets. Update package dependencies
* Install bins in correct place
* Updated Doxygen description headers
* [WIP] System Execution
* RTTFBuffer. Dictionary maps.
* [WIP] Changes to load plugins
* Changes to load plugins
* NavState refs in update
* Base interfaces for module algorithms
* Works on Map Manager Types
* Add tests for easynav_core base classes
* Group common method functionality
* Add dummy plugins for core modules
* Install runtime binary targets under lib
* Add base abstract classes for the algorithm plugins
* Refactoring to nodes
* Minor typo in license
* Add Result Class
* Reading params to core
* Separation ROS and non-ROS and skeletons
* Initial skeleton
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
