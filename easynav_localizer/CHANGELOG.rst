^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-07-26)
------------------
* Register Odometry printer in base LocalizerNode
* Remove nav_state write from dummy localizer
* Merge rolling (coverage changes) into refactor_perception
* fix(tests): fix node destructors and add comprehensive tests for controller/planner/localizer/maps_manager
  - Fix != to == in lifecycle transition guards in all 4 node destructors
  - Fix MapsManagerNode destructor: declare_parameter -> get_parameter with has_parameter guard
  - Fix unloadLibraryForClass to use actual plugin class name (from .plugin param) not sensor name
  - Null method/manager pointers before calling unload to avoid ClassLoader SIGSEGV
  - Expand inline catch blocks to satisfy uncrustify style
  - Add 11 tests for ControllerNode, 11 for PlannerNode, 12 for LocalizerNode, 10 for MapsManagerNode
  - All tests cover: node name, lifecycle transitions, plugin loading success/failure, cycle methods
* GPLv3 -> Apache 2.0
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
* Merge rolling features into kilted
* Cleanup unused headers
* Reshape execution and sensor handling
* Make dummies' fake processing time independent from clock source
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.2.0 (2025-12-01)
------------------
* Cleanup unused headers

0.1.4 (2025-10-16)
------------------
* Merge kilted version bump into rolling
* Merge kilted into rolling. Update version to 0.1.3
* Contributors: Francisco Miguel Moreno

0.1.3 (2025-10-12)
------------------
* New Constructor for PointPerceptionsView with only one perception
* Fix tf2_ros deprecation warnings
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Reformating package.xml
* Update authors and mantainers
* New CLI verb to list the plugins
* Change plugins names
* Multi-robot support by namespacing TFs
* Use tf_prefix instead of tf_namespace
* [WIP] Blackboard for NavSate
* Redesign with NavState
* Improving concurrency with atomic actions
* Yaets traces
* Initialize and default dummy cycle times to zero
* Fix CMake targets. Update package dependencies
* Install bins in corect place
* Fix ament_target_dependencies_deprecation
* Complete Dummey Controllers
* Updated Doxygen description headers
* [WIP] System Execution
* Fix CI
* Merge branch 'rolling' into goal_system
* Load controllers
* RTTFBuffer. Dictionary maps.
* WIP task synchronizations
* [WIP] Changes to load plugins
* Changes to load plugins
* NavState refs in update
* Base interfaces for module algorithms
* Add destructor and rt/nort cycle methods
* Group common method functionality
* Add dummy plugins for core modules
* Rename namespaces to easynav
* Fix typo in copyright messages
* Execute and activate nodes
* Execute and activate nodes
* Refactoring to nodes
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
