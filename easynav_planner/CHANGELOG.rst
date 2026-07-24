^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fix tf2_ros deprecation warnings and remove warnings from unused parameters
* Remove unused variables and unused-parameter warnings
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Change plugins names
* Trigger planner if new goals appear
* Use tf_prefix instead of tf_namespace
* Multi-robot support by namespacing TFs
* [WIP] Blackboard for NavSate
* Redesing with NavState
* Improving concurrency with atomic actions
* Merge branch 'rolling' into atomic_perceptions
* Blackboard working
* Improving concurrency with atomic actions
* Completed, but segfaults
* Yaets traces
* Initialize and default dummy cycle times to zero
* Fix CMake targets. Update package dependencies
* Install bins in corect place
* Fix ament_target_dependencies_deprecation
* Complete Dummey Controllers
* Fix frame id when sending goal via topic
* Updated Doxygen description headers
* [WIP] System Execution
* Load controllers
* WIP task synchronizations
* [WIP] Changes to load plugins
* Changes to load plugins
* NavState refs in update
* Works on Map Manager Types
* Add destructor and rt/nort cycle methods
* Group common method functionality
* Add dummy plugins for core modules
* Rename namespaces to easynav
* Fix typo in copyright messages
* Execute and activate nodes
* Refactoring to nodes
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, Juan Carlos Manzanares Serrano
