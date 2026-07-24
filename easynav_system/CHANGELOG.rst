^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package easynav_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2025-12-18)
------------------
* Hotfix: Remove remaining C++20/23 features
* Contributors: Francisco Miguel Moreno

0.3.1 (2025-12-17)
------------------
* Goal tolerances to the nav_state
* TF Refactor
* Unify TF configuration
* Contributors: Francisco Martín Rico, Francisco Miguel Moreno

0.3.0 (2025-12-01)
------------------
* Merge rolling features into kilted
* Set collision checker disabled by default
* Cleanup unused headers
* Reshape execution and sensor handling
* Change spin_some to spin_all. Add param for max spin time
* Dedicated thread for TF updates
* Fix TF compilation warnings
* GoalManager: Split position tolerance into x/y and height
* Remove use_sim_time param from tf node left in `#59 <https://github.com/EasyNavigation/EasyNavigation/issues/59>`_
* Fix block
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
* Fix tf2_ros deprecation warnings and remove warnings from unused parameters
* Remove unused variables and unused-parameter warnings
* Fix tf2_ros deprecation warnings
* Contributors: Francisco Miguel Moreno Olivo, Francisco Martín Rico

0.1.2 (2025-09-26)
------------------
* Reformating package.xml
* Update authors and mantainers
* Added GoalManagerInfo publishing. Navigation Control in TUI completed
* Trigger planner if new goals appear
* Select of use stamped cmd_vel
* Use tf_prefix instead of tf_namespace
* Multi-robot support by namespacing TFs
* Avoid publish cmd_vel if not set
* Add option to disable RT
* Run RT Thread in normal priority if no permissions
* Simplify blackboard with two operations
* Use ptrs for blackboard
* Refactor complete
* Blackboard redesign with NavState
* Blackboard for NavState
* Blackboard working
* Improving concurrency with atomic actions
* Fix CMake targets. Update package dependencies
* Install binaries in correct place
* Stop at idle once
* Change frequency to something reasonable
* Management of reaching goals
* Add doxygen
* Add yaets for tracing
* Check goal manager state
* Fix frame id when sending goal via topic
* Added cmd_vel
* Goal Manager first version finished
* NavigationControl interface and GoalManager
* Initial skeleton for GoalManager
* RTTFBuffer. Dictionary maps.
* WIP task synchronizations
* Changes to load plugins
* Add destructor and rt/nort cycle methods
* Add NavState structure
* Rename namespaces to easynav
* Execute and activate nodes
* Refactoring to nodes

Contributors: Francisco Martín Rico, Francisco Miguel Moreno Olivo, José Miguel Guerrero, Juan Carlos Manzanares Serrano
