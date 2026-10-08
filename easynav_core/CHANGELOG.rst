^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-10-08)
------------------
* Lifecycle states and runtime reconfiguration of the plugins (PluginSwitcher)
* Robot limits and velocity smoother; velocity multiplexer between the controller and the recovery system
* Recovery framework (RecoveryManagerBase)
* Safety mode support: configuration checks, safety channel status, data age
* Each component runs at its own frequency without drift, and whether it keeps it is reported as a WARN diagnostic (<plugin>.rt_rate / <plugin>.rate)
* The collision checker is no longer part of the controllers
* Builds on Humble, Jazzy, Kilted, Lyrical and Rolling
* Conda packages with pixi (pixi-build-ros)
* Contributors: Francisco Martín Rico

0.4.2 (2026-07-26)
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
* Add a base_footprint frame in TFInfo
* Hotfix: Remove remaining C++20/23 features
* Remove std::expected from MethodBase::initialize interface
* Added operation to get latest perception, with valid TF or not
* TFInfo in RTTFBuffer
* Create header for TFInfo
* Tests and refactoring
* TF Refactor
* Add README.md with base classes description
* Remove unused include and random
* Set collision checker disabled by default
* Comment unused variables
* Cleanup unused headers
* Add warning when exceeding the target cycle time
* Merge new changes into rolling
* Minor linting fix
* Finished collision checker
* Tracing for eficiency
* First proposal for collision checker
* Second simple approach
* First functional version
* Reset method execution time when triggered
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

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
