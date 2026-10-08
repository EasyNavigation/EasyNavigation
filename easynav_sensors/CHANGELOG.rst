^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-10-08)
------------------
* Better protection of sensor data, and perceptions invalidated after forget_time
* Runtime reconfiguration support
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Conda packages with pixi (pixi-build-ros)
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.4.2 (2026-07-26)
------------------
* Prevent possible race condition in PointPerceptions
* Fix out-of-bounds error in SensorsNode
* Add Odometry perception handler to sensors
* Fix bug when using multiple sensors
* Updating tests
* Added get_no_group in NavState
* get_by_type and get_to_vector in NavState
* Add default group. Add debug info
* PerceptionHandler: Store parent node as weak_ptr
* Merge rolling (coverage changes) into refactor_perception
* Refactor PerceptionHandler
  PerceptionHandler now represents a single sensor input, not a sensor group
* Increase coverage
* Configure failure propagation
* Mantain Perception Handler per sensor
* Add sensors' PerceptionHandler as plugins
* Merge branch 'rolling' into refactor_perception
* Perception types and ops to easynav_sensors
* GPLv3 -> Apache 2.0
* Improve navstate print including the time
* Update tests
* Add a bas_footprint frame in TFInfo
* Hotfix: Remove remaining C++20/23 features
* Buffer in pointperceptions
* TFInfo in RTTFBuffer
* Final adjustments
* TF Refactor
* Add vision_msgs/msg/Detection3DArray perception
* Cleanup unused headers
* Remove queue_size param, set queues to 1
* Fix typo in variable name
* Finished collision checker
* Lazy update in PointPerceptions
* Refactor set_by_group to avoid runtime lookups
* Add queue_size parameter
* GNSS Support
* Fix TF compilation warnings
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno, jagutic

0.1.4 (2025-10-16)
------------------
* Fix compilation errors in jazzy
* Merge kilted version bump into rolling
* Merge kilted into rolling. Update version to 0.1.3
* Contributors: Francisco Miguel Moreno

0.1.3 (2025-10-12)
------------------
* New Constructor for PointPerceptionsView with only one perception
* Fix tf2_ros deprecation warnings and remove warnings from unused parameters
* Remove unused variables and unused-parameter warnings
* Fix tf2_ros deprecation warnings
* Fix multiple perceptions
* Update defaul group var name
* Support for multiple groups
* Fix support for different perceptions types in blackboard
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Multi-robot support by namespacing TFs
* Use tf_prefix instead of tf_namespace
* [WIP] Blackboard for NavSate
* Remove atomics
* Refactor Perceptions
* BlacKboard v3
* Redesing with NavState
* Improving concurrency with atomic actions
* Yaets traces
* Fix CMake targets. Update package dependencies
* Install bins in corect place
* Fix ament_target_dependencies_deprecation
* Complete Dummey Controllers
* Updated Doxygen description headers
* Load controllers
* RTTFBuffer. Dictionary maps.
* WIP task synchronizations
* Deactivate problem test
* Improve PerceptionsOps to PerceptionsOpsView
* Add fused_perceptions method
* Add chained operations in perceptions
* Group Perceptions and their subscriptions in a single structure
* Isolate Perceptions type from sensor utility functions
* Add destructor and rt/nort cycle methods
* Sensor fusion ready
* Rename namespaces to easynav
* Conversion from laserScan
* Fix linter errors
* Fix typo in copyright messages
* [WIP] Maps manager - MapsManagerBase
* Added LaserScan
* Execute and activate nodes
* Initial sensors works
* Refactoring to nodes
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
